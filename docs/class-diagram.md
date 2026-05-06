# ATC System Class Diagram

This version uses plain ASCII boxes so it is easy to read directly in Markdown.

Note: some important project types are declared as `struct`s rather than `class`es. In C++, they are still object types, so they are included here where they matter to the design.

## Overall Structure

```text
+--------------------+         owns          +--------------------+
|       Engine       |---------------------->|         UI         |
+--------------------+                       +--------------------+
| - currentState     |                       | + DrawMainMenu()   |
| - ui               |                       | + DrawSettings...  |
| - sim              |         owns          | + DrawSimulation() |
| - selectedAircraft |------------------+    | + DrawPauseMenu()  |
| - settings         |                  |    | + DrawAirport()    |
| - debugEnabled     |                  |    | + DrawAircraft()   |
+--------------------+                  |    +--------------------+
| + init()           |                  |
| + update()         |                  |
| + render()         |                  |    +--------------------+
| + run()            |                  +--->|     Simulation     |
| + shouldClose()    |                       +--------------------+
+--------------------+                       | - aircraft         |
                                             | - airports         |
                                             | - spawnService     |
                                             | - settings         |
                                             | - lastSpawnResult  |
                                             +--------------------+
                                             | + update()         |
                                             | + requestSpawn()   |
                                             | + issueCommand()   |
                                             | + detectConflicts()|
                                             | + addAirport()     |
                                             | + getAircraftAt()  |
                                             +--------------------+
```

## Backend Domain Model

```text
+----------------------+      contains many      +----------------------+
|      Simulation      |------------------------>|       Aircraft       |
+----------------------+                         +----------------------+
| - aircraft           |                         | - performance        |
| - airports           |                         | - motion             |
| - spawnService       |                         | - command            |
| - landedCount        |                         | - callsign           |
| - outOfBoundsCount   |                         | - phase              |
+----------------------+                         | - controlMode        |
| + update()           |                         | - conflictAlert      |
| + issueCommand()     |                         | - approachCleared    |
| + toggleApproach...  |                         | - assignedIls...     |
| + detectConflicts()  |                         | - trailPoints        |
| + getGuidance...     |                         +----------------------+
| + predictTrajectory()|                         | + applyCommand()     |
+----------------------+                         | + update()           |
                                                 | + distanceTo()       |
                                                 | + breachesSepar...   |
                                                 +----------------------+


+----------------------+      contains many      +----------------------+
|      Simulation      |------------------------>|       Airport        |
+----------------------+                         +----------------------+
                                                 | + name               |
                                                 | + position           |
                                                 | + runwayHeading      |
                                                 | + runwayLength       |
                                                 | + localizer          |
                                                 +----------------------+
                                                 | + inLocalizerSignal()|
                                                 +----------------------+
```

## Aircraft Internals

```text
+------------------------+
|        Aircraft        |
+------------------------+
| - performance          |----+
| - motion               |--+ |
| - command              |-+| |
| - callsign             | || |
| - phase                | || |
| - controlMode          | || |
| - conflictAlert        | || |
| - approachCleared      | || |
| - assignedIlsAirport...| || |
+------------------------+ || |
| + applyCommand()       | || |
| + update()             | || |
| + distanceTo()         | || |
| + altitudeDifference() | || |
| + breachesSeparation() | || |
+------------------------+ || |
                           || |
                           || +--------------------------+
                           || |   AircraftPerformance    |
                           || +--------------------------+
                           || | + minSpeedKts            |
                           || | + maxSpeedKts            |
                           || | + accelerationKtsPerSec  |
                           || | + climbRateFpm           |
                           || | + descentRateFpm         |
                           || | + bankAngleDegrees       |
                           || | + minTurnRateDegPerSec   |
                           || | + maxTurnRateDegPerSec   |
                           || | + altitudeCaptureTol...  |
                           || +--------------------------+
                           ||
                           |+--------------------------+
                           ||   AircraftMotionState    |
                           |+--------------------------+
                           || + position               |
                           || + velocity               |
                           || + heading                |
                           || + speed                  |
                           || + altitude               |
                           || + verticalSpeedFpm       |
                           || + turnRateDegPerSec      |
                           || + turnRadiusNm           |
                           |+--------------------------+
                           ||
                           +--------------------------+
                           |     AircraftCommand      |
                           +--------------------------+
                           | + targetHeading          |
                           | + targetSpeed            |
                           | + targetAltitude         |
                           | + source                 |
                           +--------------------------+
```

## Navigation And Approach Model

```text
+----------------------+        has         +----------------------+
|       Airport        |------------------->|      Localizer       |
+----------------------+                    +----------------------+
| + name               |                    | + length             |
| + position           |                    | + sectors            |
| + runwayHeading      |                    +----------------------+
| + runwayLength       |                    | + isWithinSignal()   |
| + localizer          |                    +----------------------+
+----------------------+                               |
                                                       |
                                                       | has many
                                                       v
                                           +----------------------+
                                           |   LocalizerSector    |
                                           +----------------------+
                                           | + range              |
                                           | + width              |
                                           +----------------------+
```

## Service Classes

```text
+----------------------+         creates         +----------------------+
|     SpawnService     |------------------------>|    SpawnCandidate    |
+----------------------+                         +----------------------+
| - rng                |                         | + position           |
+----------------------+                         | + headingDeg         |
| + createCandidates() |                         | + speedKts           |
| - buildEntryPoints() |                         | + altitudeFt         |
| - buildCandidate()   |                         | + callsign           |
| - generateCallsign() |                         | + entryLabel         |
| - sampleHeading()    |                         +----------------------+
+----------------------+


+----------------------+       predicts        +----------------------+
| TrajectoryPredictor  |---------------------->| PredictedAircraft... |
+----------------------+                       +----------------------+
| + predict()          |                       | + timeSeconds        |
+----------------------+                       | + motion             |
                                               +----------------------+


+----------------------+      builds          +----------------------+
| GuidancePreviewSvc   |--------------------->|   GuidancePreview    |
+----------------------+                      +----------------------+
| + build()            |                      | + headingVector      |
+----------------------+                      | + turnArc            |
          |                                   | + altitudeCapture    |
          | uses                              +----------------------+
          v
+----------------------+
| TrajectoryPredictor  |
+----------------------+
```

## Enums And Shared Types

```text
+----------------------+
|      GameState       |
+----------------------+
| MENU                 |
| RUNNING              |
| PAUSED               |
| GAME_OVER            |
| EXIT                 |
+----------------------+

+----------------------+
| AircraftControlMode  |
+----------------------+
| AUTONOMOUS           |
| MANUAL               |
| ILS                  |
+----------------------+

+----------------------+
|     FlightPhase      |
+----------------------+
| ARRIVAL              |
| VECTORING            |
| ON_FINAL             |
| LANDING              |
| EXITED               |
+----------------------+

+----------------------+
|        Vec2          |
+----------------------+
| + x                  |
| + y                  |
+----------------------+
```

## Relationship Summary

```text
Engine
  -> owns UI
  -> owns Simulation
  -> stores pointer to selected Aircraft

Simulation
  -> contains many Aircraft
  -> contains many Airport
  -> uses SpawnService for traffic generation
  -> uses TrajectoryPredictor for future path checks
  -> uses GuidancePreviewService for visual guidance data

Aircraft
  -> has AircraftPerformance
  -> has AircraftMotionState
  -> has AircraftCommand
  -> uses FlightPhase and AircraftControlMode

Airport
  -> has Localizer

Localizer
  -> has many LocalizerSector
```

## Why These Classes Matter

- `Engine` is the top-level controller for menus, settings, rendering, and the application loop.
- `Simulation` is the main backend model where aircraft are updated, conflicts are checked, and ILS behaviour is applied.
- `Aircraft` is the core domain object because each plane has its own command state, motion state, and flight phase.
- `UI` is separated from backend logic so the program can draw state without owning the state changes.
- `SpawnService`, `TrajectoryPredictor`, and `GuidancePreviewService` are service/helper classes that keep specialist logic out of `Simulation`.
- `Airport`, `Localizer`, and `LocalizerSector` model the runway approach system needed for autonomous landings.
- `AircraftCommand`, `AircraftMotionState`, and `AircraftPerformance` matter because they separate controller intent, live aircraft state, and physical limits.

## Class Evidence In The Code

- `Engine`: `src/app/engine.h`, `src/app/engine.cpp`
- `UI`: `src/frontend/ui.h`, `src/frontend/ui.cpp`
- `Simulation`: `src/backend/Simulation.h`, `src/backend/Simulation.cpp`
- `Aircraft`: `src/backend/Aircraft.h`, `src/backend/Aircraft.cpp`
- `SpawnService`: `src/backend/SpawnService.h`, `src/backend/SpawnService.cpp`
- `TrajectoryPredictor`: `src/backend/TrajectoryPredictor.h`, `src/backend/TrajectoryPredictor.cpp`
- `GuidancePreviewService`: `src/backend/GuidancePreview.h`, `src/backend/GuidancePreview.cpp`
- `Airport`, `Localizer`, `LocalizerSector`: `src/backend/airport.h`, `src/backend/airport.cpp`
- `AircraftCommand`, `AircraftControlMode`, `FlightPhase`: `src/backend/AircraftControl.h`
- `AircraftMotionState`: `src/backend/AircraftMotion.h`, `src/backend/AircraftMotion.cpp`
- `AircraftPerformance`: `src/backend/AircraftPerformance.h`
- `Vec2`: `src/common/utils.h`
- `GameState`, `AppSettings`, `SimSettings`: `src/common/constants.h`
