/*
   profiler.cpp
   Copyright (C) 2020 Luke Benstead
   Copyright (C) 2023, 2024 Ruslan Rostovtsev
*/

#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include <kos/thread.h>
#include <dc/fs_dcload.h>

#include <arch/timer.h>
#include <kos/thread.h>
#include <dc/perfctr.h>

perf_cntr_event_t profilerEvent = PMCR_PARALLEL_INSTRUCTION_ISSUED_MODE;
// perf_cntr_event_t profilerMode = PMCR_PIPELINE_FREEZE_BY_DCACHE_MISS_MODE;

FILE *kernel_file = NULL;
static char KERNEL_OUTPUT_FILENAME[128];
static char SECOND_OUTPUT_FILENAME[128];
static kthread_t* THREAD;
static volatile bool PROFILER_RUNNING = false;
static volatile bool PROFILER_RECORDING = false;
// static tid_t KERNEL_TID = 0;
// static tid_t SECOND_TID = 0;
static int prof_count = 0;

#define BASE_ADDRESS 0x8c010000
#define BUCKET_SIZE 10000

#define INTERVAL_IN_MS 1

/* Simple hash table of samples. An array of Samples
 * but, each sample in that array can be the head of
 * a linked list of other samples */
typedef struct Arc {
    uint32_t pc;
    uint32_t pr; // Caller return address
    uint32_t count;
    // tid_t tid;
    struct Arc* next;
} Arc;

static Arc ARCS[BUCKET_SIZE];

/* Hashing function for two uint32_ts */
#define HASH_PAIR(x, y) ((x * 0x1f1f1f1f) ^ y)

#define BUFFER_SIZE (1024 * 64)  // 64K buffer

const static size_t MAX_ARC_COUNT = BUFFER_SIZE / sizeof(Arc);
Arc arcbuf[MAX_ARC_COUNT];

static size_t ARC_COUNT = 0;

static bool write_samples(tid_t tid, FILE *out);
static void clear_samples();

static Arc* new_arc(tid_t tid, uint32_t PC, uint32_t PR, uint32_t counter) {
    Arc* s = &arcbuf[ARC_COUNT];
    s->count = counter;
    s->pc = PC;
    s->pr = PR;
    // s->tid = tid;
    s->next = NULL;

    ++ARC_COUNT;

    return s;
}

static void record_thread(tid_t tid, uint32_t PC, uint32_t PR, uint32_t counter) {
    uint32_t bucket = HASH_PAIR(PC, PR) % BUCKET_SIZE;

    Arc* s = &ARCS[bucket];
    // profileTick = (profileTick*3 + counter)/4;

    if(s->pc) {
        /* Initialized sample in this bucket,
         * does it match though? */
        while(s->pc != PC || s->pr != PR) {
            if(s->next) {
                s = s->next;
            } else {
                s->next = new_arc(tid, PC, PR, counter);
                return; // We're done
            }
        }

        s->count+=counter;
    } else {
        /* Initialize this sample */
        s->count = counter;
        s->pc = PC;
        s->pr = PR;
        // s->tid = tid;
        s->next = NULL;
        // ++ARC_COUNT;
    }
}



#define GMON_COOKIE "gmon"
#define GMON_VERSION 1

typedef struct {
    char cookie[4];  // 'g','m','o','n'
    int32_t version; // 1
    char spare[3 * 4]; // Padding
} GmonHeader;

typedef struct {
    uint32_t low_pc;
    uint32_t high_pc;
    uint32_t hist_size;
    uint32_t prof_rate;
    char dimen[15];			/* phys. dim., usually "seconds" */
    char dimen_abbrev;			/* usually 's' for "seconds" */
} GmonHistHeader;

typedef struct {
    unsigned char tag; // GMON_TAG_TIME_HIST = 0, GMON_TAG_CG_ARC = 1, GMON_TAG_BB_COUNT = 2
    size_t ncounts; // Number of address/count pairs in this sequence
} GmonBBHeader;

typedef struct {
    uint32_t from_pc;	/* address within caller's body */
    uint32_t self_pc;	/* address within callee's body */
    uint32_t count;			/* number of arc traversals */
} GmonArc;

static bool init_sample_file(const char* path) {

    kernel_file = fopen(path, "w");
    if(!kernel_file) {
        return false;
    }

    /* Write the GMON header */

    GmonHeader header;
    memcpy(&header.cookie[0], GMON_COOKIE, sizeof(header.cookie));
    header.version = 1;
    memset(header.spare, '\0', sizeof(header.spare));

    fwrite(&header, sizeof(header), 1, kernel_file);

    return true;
}

#define ROUNDDOWN(x,y) (((x)/(y))*(y))
#define ROUNDUP(x,y) ((((x)+(y)-1)/(y))*(y))

static bool write_samples(tid_t tid, FILE *out) {
    /* Appends the samples to the output file in gmon format
     *
     * We iterate the data twice, first generating arcs, then generating
     * basic block counts. While we do that though we calculate the data
     * for the histogram so we don't need a third iteration */


    // Seek to the end of the file
    fseek(out, 0, SEEK_END);

    uint8_t tag = 1;
    size_t written = 0;

    /* Write arcs */
    Arc* root = ARCS;
    for(int i = 0; i < BUCKET_SIZE; ++i) {
        if(root->pc) {
            GmonArc arc;
            arc.from_pc = root->pr;
            arc.self_pc = root->pc;
            arc.count = root->count;

            /* Write the root sample if it has a program counter */
            fwrite(&tag, sizeof(tag), 1, out);
            fwrite(&arc, sizeof(GmonArc), 1, out);

            ++written;

            /* If there's a next pointer, traverse the list */
            Arc* s = root->next;
            while(s) {
                arc.from_pc = s->pr;
                arc.self_pc = s->pc;
                arc.count = s->count;

                /* Write the root sample if it has a program counter */
                fwrite(&tag, sizeof(tag), 1, out);
                fwrite(&arc, sizeof(GmonArc), 1, out);

                ++written;
                s = s->next;
            }
        }

        root++;
    }


    dbglog(DBG_INFO, "-- Written %d arcs\n", written);

    return true;
}

static void prof_thd_timer_hnd(irq_context_t *context) {

    if (PROFILER_RECORDING) {
        record_thread(1, context->pc, context->pr, 1);

        if(ARC_COUNT >= MAX_ARC_COUNT) {
            PROFILER_RECORDING = 0;
        }
    }

    // replicate thd timer behaviour
    {
        /* Get the system time */
        uint64_t now = timer_ms_gettime64();

        (void)context;

        //printf("timer woke at %d\n", (uint32_t)now);

        thd_schedule(0, now);
        timer_primary_wakeup(PROFILER_RECORDING ? 1 : 10);
    }
}


static void* run(void* args) {
    dbglog(DBG_INFO, "-- Entered profiler thread!\n");

    while(PROFILER_RUNNING){
        auto mask = irq_disable();
        if(!PROFILER_RECORDING) {
            if (ARC_COUNT > 0) {
                if(!write_samples(1, kernel_file)) {
                    dbglog(DBG_ERROR, "Error writing samples\n");
                }
                fclose(kernel_file);
                kernel_file = nullptr;
                clear_samples();
                dbglog(DBG_INFO, "-- Profiler thread stopped recording\n");
            }
        } else {
            uint64_t counter = perf_cntr_count(PRFC1);
            assert(counter <= UINT32_MAX);

            perf_cntr_clear(PRFC1);
            
            perf_cntr_resume(PRFC1);

            dbglog(DBG_INFO, "-- Profiler thread recording..., %d arcs, %u events\n", ARC_COUNT, (unsigned)counter);
        }
        irq_restore(mask);

        usleep(1000 * 1000); //usleep takes microseconds
    }

    dbglog(DBG_INFO, "-- Profiler thread finished!\n");

    return NULL;
}

void profiler_init(const char* output) {
    
    timer_primary_set_callback(prof_thd_timer_hnd);
    
    /* Store the filenames */
    sprintf(KERNEL_OUTPUT_FILENAME, "%s/kernel_gmon_%d.out", output, ++prof_count);
    // sprintf(SECOND_OUTPUT_FILENAME, "%s/second_gmon_%d.out", output, prof_count);

    /* Initialize the file */
    dbglog(DBG_INFO, "Creating profiler samples file for kernel thread...\n");
    if(!init_sample_file(KERNEL_OUTPUT_FILENAME)) {
        dbglog(DBG_ERROR, "Can't create %s\n", KERNEL_OUTPUT_FILENAME);
        return;
    }

    // dbglog(DBG_INFO, "Creating profiler samples file for video thread...\n");
    // if(!init_sample_file(SECOND_OUTPUT_FILENAME)) {
    //     dbglog(DBG_ERROR, "Can't create %s\n", SECOND_OUTPUT_FILENAME);
    //     return;
    // }

    dbglog(DBG_INFO, "Creating profiler thread...\n");
    // Initialize the samples to zero
    memset(ARCS, 0, sizeof(ARCS));

    PROFILER_RUNNING = true;
    THREAD = thd_create(0, run, NULL);

    // /* Lower priority is... er, higher */
    thd_set_prio(THREAD, PRIO_DEFAULT / 2);

    dbglog(DBG_INFO, "Profiler thread started.\n");
}

void profiler_start() {
    assert(PROFILER_RUNNING);

    if(PROFILER_RECORDING) {
        return;
    }

    auto mask = irq_disable();
    dbglog(DBG_INFO, "Starting profiling...\n");
    if (profilerEvent != PMCR_INIT_NO_MODE) {
        dbglog(DBG_INFO, "Using event profiling...\n");
        perf_cntr_start(PRFC1, profilerEvent, PMCR_COUNT_CPU_CYCLES);
    }
    PROFILER_RECORDING = true;
    clear_samples();
    irq_restore(mask);
}

static void clear_samples() {
    /* Free the samples we've collected to start again */

    // Wipe the lot
    memset(ARCS, 0, sizeof(ARCS));
    ARC_COUNT = 0;
}

bool profiler_stop() {
    if(!PROFILER_RECORDING) {
        return false;
    }

    bool rv = true;

    auto mask = irq_disable();

    if (PROFILER_RECORDING) {
        dbglog(DBG_INFO, "profiler_stop: Stopping profiling...\n");

        PROFILER_RECORDING = false;

        if(!write_samples(1, kernel_file)) {
            dbglog(DBG_ERROR, "ERROR WRITING SAMPLES (RO filesystem?)!\n");
            rv = false;
        }
        clear_samples();
        fclose(kernel_file);
        kernel_file = nullptr;
    }
    irq_restore(mask);

    return rv;
}

bool profiler_recording() {
    return PROFILER_RECORDING;
}

void profiler_clean_up() {
    profiler_stop(); // Make sure everything is stopped

    PROFILER_RUNNING = false;
    thd_join(THREAD, NULL);

    if(kernel_file != NULL) {
        fclose(kernel_file);
    }
}
