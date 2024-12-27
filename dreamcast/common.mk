GIT_VERSION := $(shell git describe --always --tags --long --dirty 2>/dev/null || echo "NO_GIT")
CI_JOB_ID ?= NO_CI


git-version.tmp:
	@echo "Generating git-version.tmp with GIT_VERSION = \"$(GIT_VERSION)\""
	@echo "#pragma once" > git-version.tmp
	@echo "#ifndef VERSION_H" >> git-version.tmp
	@echo "#define VERSION_H" >> git-version.tmp
	@echo "#define GIT_VERSION \"$(GIT_VERSION)\"" >> git-version.tmp
	@echo "#define CI_JOB_ID \"$(CI_JOB_ID)\"" >> git-version.tmp
	@echo "#endif // VERSION_H" >> git-version.tmp

git-version.h: git-version.tmp
	@if [ ! -f git-version.h ] || ! cmp -s git-version.tmp git-version.h; then \
	  echo "Updating git-version.h"; \
	  cp git-version.tmp git-version.h; \
    else \
	  echo "git-version.h is up to date. No change."; \
	fi

.PHONY: git-version.tmp

../src/skel/dc/dc.cpp: git-version.h


# List all of your C files here, but change the extension to ".o"
# Include "romdisk.o" if you want a rom disk.
RE3_OBJS = \
	../src/animation/AnimBlendAssocGroup.o \
	../src/animation/AnimBlendAssociation.o \
	../src/animation/AnimBlendClumpData.o \
	../src/animation/AnimBlendHierarchy.o \
	../src/animation/AnimBlendNode.o \
	../src/animation/AnimBlendSequence.o \
	../src/animation/AnimManager.o \
	../src/animation/Bones.o \
	../src/animation/CutsceneMgr.o \
	../src/animation/FrameUpdate.o \
	../src/animation/RpAnimBlend.o \
	\
	../src/buildings/Building.o \
	../src/buildings/Treadable.o \
	\
	../src/collision/ColBox.o \
	../src/collision/ColLine.o \
	../src/collision/Collision.o \
	../src/collision/ColModel.o \
	../src/collision/ColPoint.o \
	../src/collision/ColSphere.o \
	../src/collision/ColTriangle.o \
	../src/collision/TempColModels.o \
	../src/collision/VuCollision.o \
	\
	../src/control/AutoPilot.o \
	../src/control/Bridge.o \
	../src/control/CarAI.o \
	../src/control/CarCtrl.o \
	../src/control/Curves.o \
	../src/control/Darkel.o \
	../src/control/GameLogic.o \
	../src/control/Garages.o \
	../src/control/NameGrid.o \
	../src/control/OnscreenTimer.o \
	../src/control/PathFind.o \
	../src/control/Phones.o \
	../src/control/Pickups.o \
	../src/control/PowerPoints.o \
	../src/control/Record.o \
	../src/control/Remote.o \
	../src/control/Replay.o \
	../src/control/Restart.o \
	../src/control/RoadBlocks.o \
	../src/control/SceneEdit.o \
	../src/control/Script.o \
	../src/control/Script2.o \
	../src/control/Script3.o \
	../src/control/Script4.o \
	../src/control/Script5.o \
	../src/control/Script6.o \
	../src/control/ScriptDebug.o \
	../src/control/TrafficLights.o \
	\
	../src/core/Accident.o \
	../src/core/Cam.o \
	../src/core/Camera.o \
	../src/core/CdStreamDC.o \
	../src/core/Clock.o \
	../src/core/ControllerConfig.o \
	../src/core/Debug.o \
	../src/core/Directory.o \
	../src/core/EventList.o \
	../src/core/FileLoader.o \
	../src/core/FileMgr.o \
	../src/core/Fire.o \
	../src/core/Frontend.o \
	../src/core/FrontEndControls.o \
	../src/core/Frontend_PS2.o \
	../src/core/Game.o \
	../src/core/IniFile.o \
	../src/core/Lists.o \
	../src/core/main.o \
	../src/core/MenuScreens.o \
	../src/core/MenuScreensCustom.o \
	../src/core/obrstr.o \
	../src/core/Pad.o \
	../src/core/Placeable.o \
	../src/core/PlayerInfo.o \
	../src/core/Pools.o \
	../src/core/Profile.o \
	../src/core/Radar.o \
	../src/core/Range2D.o \
	../src/core/Range3D.o \
	../src/core/re3.o \
	../src/core/References.o \
	../src/core/Stats.o \
	../src/core/Streaming.o \
	../src/core/SurfaceTable.o \
	../src/core/timebars.o \
	../src/core/Timer.o \
	../src/core/TimeStep.o \
	../src/core/User.o \
	../src/core/Wanted.o \
	../src/core/World.o \
	../src/core/ZoneCull.o \
	../src/core/Zones.o \
	\
	../src/entities/Dummy.o \
	../src/entities/Entity.o \
	../src/entities/Physical.o \
	\
	../src/fakerw/fake.o \
	\
	../src/math/math.o \
	../src/math/Matrix.o \
	../src/math/Quaternion.o \
	../src/math/Rect.o \
	../src/math/Vector.o \
	\
	../src/modelinfo/BaseModelInfo.o \
	../src/modelinfo/ClumpModelInfo.o \
	../src/modelinfo/MloModelInfo.o \
	../src/modelinfo/ModelIndices.o \
	../src/modelinfo/ModelInfo.o \
	../src/modelinfo/PedModelInfo.o \
	../src/modelinfo/SimpleModelInfo.o \
	../src/modelinfo/TimeModelInfo.o \
	../src/modelinfo/VehicleModelInfo.o \
	\
	../src/objects/CutsceneHead.o \
	../src/objects/CutsceneObject.o \
	../src/objects/DummyObject.o \
	../src/objects/Object.o \
	../src/objects/ObjectData.o \
	../src/objects/ParticleObject.o \
	../src/objects/Projectile.o \
	\
	../src/peds/CivilianPed.o \
	../src/peds/CopPed.o \
	../src/peds/EmergencyPed.o \
	../src/peds/Gangs.o \
	../src/peds/Ped.o \
	../src/peds/PedAI.o \
	../src/peds/PedChat.o \
	../src/peds/PedDebug.o \
	../src/peds/PedFight.o \
	../src/peds/PedIK.o \
	../src/peds/PedPlacement.o \
	../src/peds/PedRoutes.o \
	../src/peds/PedType.o \
	../src/peds/PlayerPed.o \
	../src/peds/Population.o \
	\
	../src/renderer/Antennas.o \
	../src/renderer/Clouds.o \
	../src/renderer/Console.o \
	../src/renderer/Coronas.o \
	../src/renderer/Credits.o \
	../src/renderer/Draw.o \
	../src/renderer/Fluff.o \
	../src/renderer/Font.o \
	../src/renderer/Glass.o \
	../src/renderer/Hud.o \
	../src/renderer/Instance.o \
	../src/renderer/Lines.o \
	../src/renderer/MBlur.o \
	../src/renderer/Particle.o \
	../src/renderer/ParticleMgr.o \
	../src/renderer/PlayerSkin.o \
	../src/renderer/PointLights.o \
	../src/renderer/RenderBuffer.o \
	../src/renderer/Renderer.o \
	../src/renderer/Rubbish.o \
	../src/renderer/Shadows.o \
	../src/renderer/Skidmarks.o \
	../src/renderer/SpecialFX.o \
	../src/renderer/Sprite.o \
	../src/renderer/Sprite2d.o \
	../src/renderer/TexList.o \
	../src/renderer/Timecycle.o \
	../src/renderer/WaterCannon.o \
	../src/renderer/WaterLevel.o \
	../src/renderer/Weather.o \
	\
	../src/rw/ClumpRead.o \
	../src/rw/Lights.o \
	../src/rw/MemoryHeap.o \
	../src/rw/MemoryMgr.o \
	../src/rw/NodeName.o \
	../src/rw/RwHelper.o \
	../src/rw/RwMatFX.o \
	../src/rw/RwPS2AlphaTest.o \
	../src/rw/TexRead.o \
	../src/rw/TexturePools.o \
	../src/rw/TxdStore.o \
	../src/rw/VisibilityPlugins.o \
	\
	../src/skel/crossplatform.o \
	../src/skel/events.o \
	../src/skel/skeleton.o \
	../src/skel/dc/dc.o \
	\
	../src/text/Messages.o \
	../src/text/Pager.o \
	../src/text/Text.o \
	\
	../src/vehicles/Automobile.o \
	../src/vehicles/Boat.o \
	../src/vehicles/CarGen.o \
	../src/vehicles/Cranes.o \
	../src/vehicles/DamageManager.o \
	../src/vehicles/Door.o \
	../src/vehicles/Floater.o \
	../src/vehicles/HandlingMgr.o \
	../src/vehicles/Heli.o \
	../src/vehicles/Plane.o \
	../src/vehicles/Train.o \
	../src/vehicles/Transmission.o \
	../src/vehicles/Vehicle.o \
	\
	../src/weapons/BulletInfo.o \
	../src/weapons/Explosion.o \
	../src/weapons/ProjectileInfo.o \
	../src/weapons/ShotInfo.o \
	../src/weapons/Weapon.o \
	../src/weapons/WeaponEffects.o \
	../src/weapons/WeaponInfo.o \
	\
	../src/audio/AudioCollision.o \
	../src/audio/AudioLogic.o \
	../src/audio/AudioManager.o \
	../src/audio/AudioScriptObject.o \
	../src/audio/DMAudio.o \
	../src/audio/MusicManager.o \
	../src/audio/PolRadio.o \
	../src/audio/sampman_miles.o \
	../src/audio/sampman_oal.o \
	\
	../src/save/Date.o \
	../src/save/GenericGameStorage.o \
	../src/save/MemoryCard.o \
	../src/save/PCSave.o \
	\
	../src/extras/debugmenu.o \
	../src/extras/frontendoption.o \
	../src/extras/postfx.o \
	../src/extras/screendroplets.o \
	\
	../src/vmu/vmu.o \
	../vendor/miniLZO/minilzo.o \
	\

# Excluded \
	../src/extras/custompipes.o \
	../src/extras/custompipes_d3d9.o \
	../src/extras/custompipes_gl.o \
	../src/core/CdStream.o \
	../src/core/CdStreamPosix.o \
	../src/extras \
	../src/extras/GitSHA1.cpp.in \
	../src/core/AnimViewer.o \

RW_OBJS = \
    ../vendor/librw/src/anim.o \
    ../vendor/librw/src/base.o \
    ../vendor/librw/src/camera.o \
    ../vendor/librw/src/charset.o \
    ../vendor/librw/src/clump.o \
    ../vendor/librw/src/engine.o \
    ../vendor/librw/src/error.o \
    ../vendor/librw/src/frame.o \
    ../vendor/librw/src/geometry.o \
    ../vendor/librw/src/geoplg.o \
    ../vendor/librw/src/hanim.o \
    ../vendor/librw/src/image.o \
    ../vendor/librw/src/light.o \
    ../vendor/librw/src/matfx.o \
    ../vendor/librw/src/pipeline.o \
    ../vendor/librw/src/plg.o \
    ../vendor/librw/src/prim.o \
    ../vendor/librw/src/raster.o \
    ../vendor/librw/src/render.o \
    ../vendor/librw/src/skin.o \
    ../vendor/librw/src/texture.o \
    ../vendor/librw/src/tristrip.o \
    ../vendor/librw/src/userdata.o \
    ../vendor/librw/src/uvanim.o \
    ../vendor/librw/src/world.o \
	\
	../vendor/librw/src/dc/rwdc.o \
	../vendor/librw/src/dc/alloc.o

# Excluded \
	../vendor/librw/src/d3d-x/d3d.o \
	../vendor/librw/src/d3d-x/d3d8.o \
	../vendor/librw/src/d3d-x/d3d8render.o \
	../vendor/librw/src/d3d/d3d8.o \
    ../vendor/librw/src/d3d/d3d8matfx.o \
    ../vendor/librw/src/d3d/d3d8render.o \
    ../vendor/librw/src/d3d/d3d8skin.o \
    ../vendor/librw/src/d3d/d3d9.o \
    ../vendor/librw/src/d3d/d3d9matfx.o \
    ../vendor/librw/src/d3d/d3d9render.o \
    ../vendor/librw/src/d3d/d3d9skin.o \
    ../vendor/librw/src/d3d/d3d.o \
    ../vendor/librw/src/d3d/d3ddevice.o \
    ../vendor/librw/src/d3d/d3dimmed.o \
    ../vendor/librw/src/d3d/d3drender.o \
    ../vendor/librw/src/d3d/xbox.o \
    ../vendor/librw/src/d3d/xboxmatfx.o \
    ../vendor/librw/src/d3d/xboxskin.o \
    ../vendor/librw/src/d3d/xboxvfmt.o \
	\
    ../vendor/librw/src/gl/gl3.o \
    ../vendor/librw/src/gl/gl3device.o \
    ../vendor/librw/src/gl/gl3immed.o \
    ../vendor/librw/src/gl/gl3matfx.o \
    ../vendor/librw/src/gl/gl3pipe.o \
    ../vendor/librw/src/gl/gl3raster.o \
    ../vendor/librw/src/gl/gl3render.o \
    ../vendor/librw/src/gl/gl3shader.o \
    ../vendor/librw/src/gl/gl3skin.o \
    ../vendor/librw/src/gl/wdgl.o \
    ../vendor/librw/src/gl/glad/glad.cXXX \
	\
    ../vendor/librw/src/ps2/pds.o \
    ../vendor/librw/src/ps2/ps2.o \
    ../vendor/librw/src/ps2/ps2device.o \
    ../vendor/librw/src/ps2/ps2matfx.o \
    ../vendor/librw/src/ps2/ps2raster.o \
    ../vendor/librw/src/ps2/ps2skin.o \

INCLUDE = \
-I../src/animation \
-I../src/audio \
-I../src/buildings \
-I../src/collision \
-I../src/control \
-I../src/core \
-I../src/entities \
-I../src/extras \
-I../src/fakerw \
-I../src/math \
-I../src/modelinfo \
-I../src/objects \
-I../src/peds \
-I../src/renderer \
-I../src/rw \
-I../src/save \
-I../src/skel \
-I../src/text \
-I../src/vehicles \
-I../src/weapons \
-I../src/audio/eax \
-I../src/audio/oal \
-I../src/extras/shaders \
-I../src/extras/shaders/obj \
-I../src/skel/glfw \
-I../src/skel/win \
\
-I../vendor/librw \
\
-I../vendor/miniLZO

DEFINES = -DRW_DC -DLIBRW $(if $(WITH_LOGGING),-DWITH_LOGGING) $(if $(WITH_DCLOAD),-DDC_CHDIR=/pc) \
	$(if $(WITH_BEEPS),-DWITH_BEEPS)
FLAGS = -fpermissive -Wno-sign-compare -Wno-parentheses -Wno-maybe-uninitialized \
	-Wno-format -Wno-strict-aliasing -Wno-unused-variable \
	-Wno-unused-but-set-variable -Wno-write-strings \
	-Wno-deprecated-enum-enum-conversion -Wno-deprecated-enum-float-conversion \
	-Wno-multichar -Wno-unused-value -Wno-char-subscripts -Wno-reorder \
	-Wno-unused-function -Wno-class-memaccess -fno-permissive

CPPFLAGS += $(INCLUDE) $(DEFINES) $(FLAGS)
CFLAGS += -std=gnu17 $(CPPFLAGS)
CXXFLAGS += -std=gnu++20 $(CPPFLAGS)