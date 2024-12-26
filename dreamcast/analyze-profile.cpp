#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <iomanip>
#include <regex>
#include <set>
#include <unordered_set>
#include <functional>
#include <cxxabi.h>  // For demangling

#define SAMPLE_PERCENTAGE_THRESHOLD 0.1  // Threshold for including functions

struct GmonHeader {
    char cookie[4];    // 'g','m','o','n'
    int32_t version;   // 1
    char spare[12];    // Padding
};

struct GmonArc {
    uint32_t from_pc;  // Address within caller's body
    uint32_t self_pc;  // Address within callee's body
    uint32_t count;    // Number of arc traversals
};

struct FunctionInfo {
    std::string mangled_name;     // Mangled function name
    std::string demangled_name;   // Demangled function name
    uint32_t start_address;
    uint32_t end_address; // Will be updated as we parse the source file
    uint64_t sample_count;
    double sample_percentage; // Percentage of total samples
    uint64_t samples_with_children; // Total samples including callees
    std::unordered_map<uint32_t, uint64_t> callers; // Set of caller addresses
};

// Updated demangle_symbol function using abi::__cxa_demangle
std::string demangle_symbol(const std::string& mangled_name) {
    static std::unordered_map<std::string, std::string> demangle_cache;

    // Check if the name is already in the cache
    auto cache_it = demangle_cache.find(mangled_name);
    if (cache_it != demangle_cache.end()) {
        return cache_it->second;
    }

    int status = 0;
    char* demangled = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);
    std::string demangled_name;
    if (status == 0 && demangled != nullptr) {
        demangled_name = demangled;
        free(demangled);
    } else {
        // If demangling fails, return the mangled name
        demangled_name = mangled_name;
    }
    demangle_cache[mangled_name] = demangled_name;
    return demangled_name;
}

int main(int argc, char* argv[]) {
    // Enable synchronization between C++ and C I/O
    std::ios::sync_with_stdio(false);

    // Check for the required arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <gmon_file> <source_dump_file>" << std::endl;
        return 1;
    }

    const char* gmon_filename = argv[1];       // Profiler output file
    const char* source_filename = argv[2];     // Source dump file

    std::cout << "[*] Starting profiling data analysis..." << std::endl;

    // Open the profiler output file
    std::cout << "[*] Reading profiler output file: " << gmon_filename << std::endl;
    std::ifstream infile(gmon_filename, std::ios::binary);
    if (!infile) {
        std::cerr << "Error opening file " << gmon_filename << std::endl;
        return 1;
    }

    // Read GmonHeader
    GmonHeader header;
    infile.read(reinterpret_cast<char*>(&header), sizeof(GmonHeader));

    // Verify the header
    if (std::strncmp(header.cookie, "gmon", 4) != 0) {
        std::cerr << "Invalid file format: missing 'gmon' cookie" << std::endl;
        return 1;
    }

    // Collect arc records
    std::vector<GmonArc> arcs;
    while (infile.peek() != EOF) {
        uint8_t tag;
        infile.read(reinterpret_cast<char*>(&tag), sizeof(tag));

        if (infile.eof()) {
            break;
        }

        if (tag == 1) {
            // Arc record
            GmonArc arc;
            infile.read(reinterpret_cast<char*>(&arc), sizeof(GmonArc));
            arcs.push_back(arc);
        } else if (tag == 0) {
            // Histogram data: skip it
            // Read GmonHistHeader to get hist_size
            struct {
                uint32_t low_pc;
                uint32_t high_pc;
                uint32_t hist_size;
                uint32_t prof_rate;
                char dimen[15];
                char dimen_abbrev;
            } hist_header;

            infile.read(reinterpret_cast<char*>(&hist_header), sizeof(hist_header));

            // Skip the histogram bins
            infile.ignore(hist_header.hist_size * sizeof(uint16_t));
        } else {
            std::cerr << "Unknown tag encountered: " << static_cast<int>(tag) << std::endl;
            break;
        }
    }
    infile.close();
    std::cout << "[*] Profiler data reading completed." << std::endl;

    // Create a mapping from self_pc to count
    std::unordered_map<uint32_t, uint64_t> address_counts;
    uint64_t total_samples = 0;
    for (const auto& arc : arcs) {
        // Sum counts for each address
        address_counts[arc.self_pc] += arc.count;
        total_samples += arc.count;
    }
    std::cout << "[*] Total samples collected: " << total_samples << std::endl;

    // Open the source dump file
    std::cout << "[*] Processing source dump file: " << source_filename << std::endl;
    std::ifstream source_file(source_filename);
    if (!source_file) {
        std::cerr << "Error opening source dump file " << source_filename << std::endl;
        return 1;
    }

    // Prepare the output annotated file
    std::string output_filename = std::string(source_filename) + ".annotated";
    std::ofstream annotated_file(output_filename);
    if (!annotated_file) {
        std::cerr << "Error creating annotated file " << output_filename << std::endl;
        return 1;
    }

    // First pass: Identify functions and their ranges
    std::cout << "[*] Identifying functions and their address ranges..." << std::endl;
    std::string line;
    FunctionInfo current_function;
    current_function.start_address = 0;
    current_function.end_address = 0;
    current_function.sample_count = 0;

    std::vector<FunctionInfo> functions;
    uint32_t last_address = 0;
    while (std::getline(source_file, line)) {
        // Regular expression to match function headers
        // Pattern: address <function_name>:
        std::regex function_header_regex(R"(^\s*([0-9a-fA-F]+)\s+<_(.*)>:)");

        std::smatch match;
        if (std::regex_search(line, match, function_header_regex)) {
            uint32_t address = 0;
            std::istringstream addr_ss(match[1]);
            addr_ss >> std::hex >> address;

            std::string function_name = match[2];

            // Use address as name if function_name is empty
            if (function_name.empty()) {
                function_name = "0x" + match[1].str();
            }

            // Finish the previous function
            if (current_function.start_address != 0) {
                current_function.end_address = last_address + 1;
                functions.push_back(current_function);
            }

            // Start a new function
            current_function.mangled_name = function_name;
            current_function.demangled_name = demangle_symbol(function_name);
            current_function.start_address = address;
            current_function.end_address = 0;
            current_function.sample_count = 0;

            last_address = address;
            continue;
        }

        // Attempt to parse an address at the beginning of the line
        std::istringstream iss(line);
        std::string address_str;
        iss >> address_str;

        // Remove any trailing colon from the address
        if (!address_str.empty() && address_str.back() == ':') {
            address_str.pop_back();
        }

        uint32_t address = 0;
        std::istringstream addr_ss(address_str);
        addr_ss >> std::hex >> address;

        if (!addr_ss.fail()) {
            last_address = address;
        }
    }

    // Finish the last function
    if (current_function.start_address != 0) {
        current_function.end_address = UINT32_MAX; // Assume it goes to the end
        functions.push_back(current_function);
    }
    std::cout << "[*] Function identification completed. Total functions found: " << functions.size() << std::endl;

    source_file.clear();
    source_file.seekg(0, std::ios::beg); // Reset the file stream

    // Map addresses to functions
    // For efficiency, we can sort the functions and use binary search
    std::sort(functions.begin(), functions.end(), [](const FunctionInfo& a, const FunctionInfo& b) {
        return a.start_address < b.start_address;
    });

    // Second pass: Calculate sample counts and percentages per function
    std::cout << "[*] Calculating sample counts per function..." << std::endl;
    for (auto& func : functions) {
        func.sample_count = 0;
        func.samples_with_children = 0; // Initialize
    }

    // Function to find a function by address
    auto find_function_by_address = [&](uint32_t address) -> FunctionInfo* {
        // Use binary search since functions are sorted by start_address
        auto func_it = std::upper_bound(functions.begin(), functions.end(), address, [](uint32_t addr, const FunctionInfo& func) {
            return addr < func.start_address;
        });

        if (func_it != functions.begin()) {
            --func_it;
            if (address >= func_it->start_address && address < func_it->end_address) {
                return &(*func_it);
            }
        }
        return nullptr; // Function not found
    };

    // Calculate sample counts per function
    for (const auto& addr_count : address_counts) {
        uint32_t address = addr_count.first;
        uint64_t count = addr_count.second;

        // Find the function containing this address
        FunctionInfo* func = find_function_by_address(address);
        if (func) {
            func->sample_count += count;
        }
    }

    // Update caller information for each function
    for (const auto& arc : arcs) {
        FunctionInfo* func = find_function_by_address(arc.self_pc);
        if (func) {
            func->callers[arc.from_pc] += arc.count;
        }
    }

    // Calculate sample percentages per function
    for (auto& func : functions) {
        func.sample_percentage = (total_samples > 0) ? (func.sample_count * 100.0) / total_samples : 0.0;
    }
    std::cout << "[*] Sample count calculation completed." << std::endl;

    // Filter out functions with sample_percentage <= 0.1%
    functions.erase(std::remove_if(functions.begin(), functions.end(), [](const FunctionInfo& func) {
        return func.sample_percentage <= SAMPLE_PERCENTAGE_THRESHOLD;
    }), functions.end());

    // Build the call graph including only functions with > 0.1% sample_percentage
    std::cout << "[*] Building the call graph including only significant functions..." << std::endl;

    // Map from FunctionInfo* to callees with arc counts
    std::unordered_map<FunctionInfo*, std::unordered_map<FunctionInfo*, uint64_t>> call_graph;

    // Process arcs to build the call graph
    for (const auto& arc : arcs) {
        uint32_t from_address = arc.from_pc;
        uint32_t self_address = arc.self_pc;

        FunctionInfo* caller_func = find_function_by_address(from_address);
        FunctionInfo* callee_func = find_function_by_address(self_address);

        if (caller_func && callee_func) {
            // Both functions must be significant (sample_percentage > 0.1%)
            if (caller_func->sample_percentage > SAMPLE_PERCENTAGE_THRESHOLD &&
                callee_func->sample_percentage > SAMPLE_PERCENTAGE_THRESHOLD) {
                // Add an edge from caller to callee
                call_graph[caller_func][callee_func] += arc.count;
            }
        }
    }
    std::cout << "[*] Call graph construction completed." << std::endl;

    // Recompute samples_with_children for each function
    std::cout << "[*] Computing samples with children for each function..." << std::endl;
    std::unordered_map<FunctionInfo*, uint64_t> samples_with_children_map;

    std::function<uint64_t(FunctionInfo*, std::unordered_set<FunctionInfo*>&)> compute_samples_with_children =
        [&](FunctionInfo* func, std::unordered_set<FunctionInfo*>& visited) -> uint64_t {
        if (samples_with_children_map.count(func)) {
            return samples_with_children_map[func];
        }

        if (visited.count(func)) {
            // Cycle detected, avoid infinite recursion
            return 0;
        }
        visited.insert(func);

        uint64_t total_samples = func->sample_count;

        if (call_graph.count(func)) {
            for (const auto& callee_entry : call_graph[func]) {
                FunctionInfo* callee = callee_entry.first;
                total_samples += compute_samples_with_children(callee, visited);
            }
        }

        visited.erase(func);
        samples_with_children_map[func] = total_samples;
        return total_samples;
    };

    for (auto& func : functions) {
        std::unordered_set<FunctionInfo*> visited;
        func.samples_with_children = compute_samples_with_children(&func, visited);
    }
    std::cout << "[*] Samples with children computation completed." << std::endl;

    // Generate the call graph in DOT format
    std::cout << "[*] Generating the call graph DOT file..." << std::endl;
    std::string dot_filename = std::string(source_filename) + ".graph.dot";
    std::ofstream dot_file(dot_filename);
    if (!dot_file) {
        std::cerr << "Error creating DOT file " << dot_filename << std::endl;
        return 1;
    }

    dot_file << "digraph CallGraph {\n";
    dot_file << "    node [shape=box];\n";

    // Write nodes with labels including samples_with_children%(samples_self%)
    for (FunctionInfo* func_ptr = functions.data(); func_ptr != functions.data() + functions.size(); ++func_ptr) {
        FunctionInfo& func = *func_ptr;

        // Prioritize demangled name; if empty, use mangled name; if empty, use address
        std::string function_label;
        if (!func.demangled_name.empty() && func.demangled_name != func.mangled_name) {
            function_label = func.demangled_name;
        } else if (!func.mangled_name.empty()) {
            function_label = func.mangled_name;
        } else {
            std::ostringstream oss;
            oss << "0x" << std::hex << func.start_address;
            function_label = oss.str();
        }

        // Label format: samples_with_children%(samples_self%)
        double samples_with_children_percentage = (total_samples > 0) ? (func.samples_with_children * 100.0) / total_samples : 0.0;
        double samples_self_percentage = (total_samples > 0) ? (func.sample_count * 100.0) / total_samples : 0.0;

        std::ostringstream label;
        label << function_label << "\\n";
        label << std::fixed << std::setprecision(2) << samples_with_children_percentage << "%";
        label << " (" << samples_self_percentage << "%)";

        // Node identifier can be function's start_address
        std::ostringstream node_id_stream;
        node_id_stream << "0x" << std::hex << func.start_address;
        std::string node_id = node_id_stream.str();

        dot_file << "    \"" << node_id << "\" [label=\"" << label.str() << "\"];\n";
    }

    // Write edges with arc counts
    for (const auto& caller_entry : call_graph) {
        FunctionInfo* caller_func = caller_entry.first;
        std::ostringstream caller_id_stream;
        caller_id_stream << "0x" << std::hex << caller_func->start_address;
        std::string caller_id = caller_id_stream.str();

        for (const auto& callee_entry : caller_entry.second) {
            FunctionInfo* callee_func = callee_entry.first;
            uint64_t arc_count = callee_entry.second;

            std::ostringstream callee_id_stream;
            callee_id_stream << "0x" << std::hex << callee_func->start_address;
            std::string callee_id = callee_id_stream.str();

            // Edge from caller_func to callee_func with label arc_count
            dot_file << "    \"" << caller_id << "\" -> \"" << callee_id << "\" [label=\"" << arc_count << "\"];\n";
        }
    }

    dot_file << "}\n";
    dot_file.close();
    std::cout << "[*] Call graph DOT file generated: " << dot_filename << std::endl;

    // Third pass: Annotate the source file
    std::cout << "[*] Annotating the source file..." << std::endl;
    source_file.clear();
    source_file.seekg(0, std::ios::beg); // Reset the file stream
    FunctionInfo* current_function_ptr = nullptr;

    while (std::getline(source_file, line)) {
        std::string original_line = line; // Keep the original line for output

        // Regular expression to match function headers
        std::regex function_header_regex(R"(^\s*([0-9a-fA-F]+)\s+<(.*)>:)");

        std::smatch match;
        if (std::regex_search(line, match, function_header_regex)) {
            // Function header line
            uint32_t address = 0;
            std::istringstream addr_ss(match[1]);
            addr_ss >> std::hex >> address;

            // Find the function info
            current_function_ptr = find_function_by_address(address);

            // Annotate the function header line if the function is significant
            if (current_function_ptr && current_function_ptr->sample_percentage > SAMPLE_PERCENTAGE_THRESHOLD) {
                uint32_t func_samples = current_function_ptr->sample_count;
                double func_percentage = current_function_ptr->sample_percentage;

                // Prioritize demangled name; if empty, use mangled name; if empty, use address
                std::string function_label;
                if (!current_function_ptr->demangled_name.empty() && current_function_ptr->demangled_name != current_function_ptr->mangled_name) {
                    function_label = current_function_ptr->demangled_name;
                } else if (!current_function_ptr->mangled_name.empty()) {
                    function_label = current_function_ptr->mangled_name;
                } else {
                    std::ostringstream oss;
                    oss << "0x" << std::hex << current_function_ptr->start_address;
                    function_label = oss.str();
                }

                annotated_file << original_line << " !!!! " << func_samples << " " << std::fixed << std::setprecision(2) << func_percentage << "% @@@ " << function_label << std::endl;
            } else {
                // Function is not significant or not found
                current_function_ptr = nullptr;
                annotated_file << original_line << std::endl;
            }

            continue;
        }

        // Attempt to parse an address at the beginning of the line
        std::istringstream iss(line);
        std::string address_str;
        iss >> address_str;

        // Remove any trailing colon from the address
        if (!address_str.empty() && address_str.back() == ':') {
            address_str.pop_back();
        }

        uint32_t address = 0;
        std::istringstream addr_ss(address_str);
        addr_ss >> std::hex >> address;

        if (addr_ss.fail()) {
            // Not a valid address, write the line as is
            annotated_file << original_line << std::endl;
            continue;
        }

        // For code lines within significant functions
        if (current_function_ptr && current_function_ptr->sample_percentage > SAMPLE_PERCENTAGE_THRESHOLD) {
            // Check if the address is in the address_counts map
            auto count_it = address_counts.find(address);
            if (count_it != address_counts.end()) {
                uint32_t sample_count = count_it->second;
                uint32_t func_samples = current_function_ptr->sample_count;
                double line_percentage = (func_samples > 0) ? (sample_count * 100.0) / func_samples : 0.0;

                annotated_file << original_line << " !!!! " << sample_count << " " << std::fixed << std::setprecision(2) << line_percentage << "%" << std::endl;
            } else {
                // No samples for this line
                annotated_file << original_line << std::endl;
            }
        } else {
            // Not within a significant function
            annotated_file << original_line << std::endl;
        }
    }
    source_file.close();
    annotated_file.close();
    std::cout << "[*] Source file annotation completed. Annotated file: " << output_filename << std::endl;

    // Generate top functions report
    std::cout << "[*] Generating top functions report..." << std::endl;

    // Sort functions by sample count in descending order
    std::sort(functions.begin(), functions.end(), [](const FunctionInfo& a, const FunctionInfo& b) {
        return a.sample_count > b.sample_count;
    });

    std::cout << "\nTop functions by sample count:" << std::endl;
    std::cout << std::setw(6) << "Rank"
              << std::setw(10) << "Samples"
              << std::setw(10) << "Percent"
              << std::setw(10) << "CumSum"
              << "  Function Name" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    double cumulative_percentage = 0.0;
    for (size_t i = 0; i < functions.size(); ++i) {
        double percent = functions[i].sample_percentage;
        cumulative_percentage += percent;

        // Prioritize demangled name; if empty, use mangled name; if empty, use address
        std::string function_label;
        if (!functions[i].demangled_name.empty() && functions[i].demangled_name != functions[i].mangled_name) {
            function_label = functions[i].demangled_name;
        } else if (!functions[i].mangled_name.empty()) {
            function_label = functions[i].mangled_name;
        } else {
            std::ostringstream oss;
            oss << "0x" << std::hex << functions[i].start_address;
            function_label = oss.str();
        }

        std::cout << std::setw(6) << (i + 1)
                  << std::setw(10) << functions[i].sample_count
                  << std::setw(9) << std::fixed << std::setprecision(2) << percent << "%"
                  << std::setw(9) << std::fixed << std::setprecision(2) << cumulative_percentage << "%"
                  << "  " << function_label << std::endl;
        #if defined(VERBOSE_CALLERS)
        for (auto &caller : functions[i].callers) {
            auto func = find_function_by_address(caller.first);
            std::cout << "    Called by 0x" << std::hex << caller.first << " " << std::dec << caller.second << " times, "  << (func ? func->demangled_name : "<unknown>") << std::endl;
        }
        #endif
    }

    std::cout << "[*] Top functions report generated." << std::endl;
    std::cout << "[*] Profiling data analysis completed." << std::endl;

    return 0;
}
