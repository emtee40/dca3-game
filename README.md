## Intro

dca3 is a port of GTA III for the Dreamcast made by The Gang, using [re3](https://github.com/halpz/re3/tree/master/) as a base.

re3 a fully reversed source code for GTA III.

This project was started by [Stefanos Kornilios Mitsis Poiitidis](https://x.com/poiitidis) and uses [KallistiOS](https://kos-docs.dreamcast.wiki/).

## Baking the CDI
### Prerequisites
You need GTA 3 installed. This version has been tested and works:  https://store.rockstargames.com/game/buy-grand-theft-auto-the-trilogy.

Please note that *SOME VERSIONS* of the game may not work. It has been reported that `d4_gta.mp3` is corrupted sometimes.

Make sure you have a LEGIT copy with no corrupted files, as this process wont work otherwise.

You will also need the following tools installed
- git-scm http://git-scm.com/downloads/win
- dreamsdk r3 https://github.com/dreamsdk/dreamsdk/releases

### Preparing the gta3 folder
- Open dreamsdk shell
- type `mkdir gta3` (and press enter)
- type `explorer .` (and press enter)
- This will open a folder named gta3. Copy your gta3 files in there.
  - If you use the 2cdrom version of the game, make sure to also copy the contents of the play disc to this folder.
- close the folder and the dreamsdk shell and proceed to the next step.

### Cloning the dca3-game repo and downloading the prebuilt elf
- Open dreamsdk shell
- type `git clone https://gitlab.com/skmp/dca3-game.git` (and press enter)
- It should take a moment and successfully clone the repo
- type `cd dca3-game/dreamcast` (and press enter)
- type `explorer .` (and press enter).
- A folder named dreamcast with some files should be open. Keep it on the side.
- Download the artifacts from https://gitlab.com/skmp/dca3-game/-/jobs/8725216645
- Open artifacts.zip and extract dca3.elf to the folder that was kept open before.
- Close the folder and dreamsdk shell

### Repacking and making a prebuilt cdi FOR GD-EMU
- Open dreamsdk shell
- type `cd dca3-game/dreamcast` (and press enter)
- type `make cdi-prebuilt` (and press enter)
  - This should take a while (5-15 mins)
  - Due to an issue with dreamsdk, this won't fully complete the first time
- type `make cdi-prebuilt` (and press enter)
  - It will continue where it left off before
  - It should run to completion now and show "*** Repack Completed Successfully ***"
- type `explorer .` (and press enter)
- The dreamcast folder should open up, and it should contain dca3.cdi for you (~ 900 megs)

### Repacking and making a prebuilt cdi FOR burning CD-ROM
- Open dreamsdk shell
- type `cd dca3-game/dreamcast` (and press enter)
- type `make FOR_DISC=1 cdi-prebuilt` (and press enter)
  - This should take a while (5-15 mins)
  - Due to an issue with dreamsdk, this won't fully complete the first time
- type `make FOR_DISC=1 cdi-prebuilt` (and press enter)
  - It will continue where it left off before
  - It should run to completion now and show "*** Repack Completed Successfully ***"
- type `explorer .` (and press enter)
- The dreamcast folder should open up, and it should contain dca3.cdi for you (~ 700 megs)
- If the .cdi is not 700 megabytes, then you did something wrong.
  - You can type `rm -rf repack-data` (and press enter)
  - And then start this step from the beggining


## How to report issues
- Take a photo of your tv/monitor and vmu
- open a ticket via https://gitlab.com/skmp/dca3-game/-/issues/new
- state which elf you have used (eg, https://gitlab.com/skmp/dca3-game/-/jobs/8725216645)
- write something descriptive of what is/went wrong

## License

The code should only be used for educational, documentation and modding purposes.\
We do not encourage piracy or commercial use.\
Please keep derivate work open source and give proper credit.