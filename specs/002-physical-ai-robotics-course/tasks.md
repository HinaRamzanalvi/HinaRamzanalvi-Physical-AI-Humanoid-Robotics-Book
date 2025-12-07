# Tasks: Physical AI & Humanoid Robotics Course

**Input**: Design documents from `/specs/002-physical-ai-robotics-course/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Book project**: `book/` at repository root
- **Code examples**: `code_examples/` at repository root
- **Environments**: `environments/` at repository root

## Phase 1: Setup (Project Initialization)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `book/` directory and initialize Docusaurus project `book/`
- [x] T002 Create `code_examples/` directory `code_examples/`
- [x] T003 Create `environments/` directory `environments/`
- [x] T004 Configure basic Git ignored files for `book/`, `code_examples/`, and `environments/` in `.gitignore`
- [x] T005 [P] Set up Docusaurus configuration for the book title, navbar, and sidebar `book/docusaurus.config.js`
- [x] T006 [P] Create initial Docusaurus `docs/` directory and `_category_.json` for Module 1 `book/docs/module1_ros2/`

---

## Phase 2: Foundational (Core Environment Setup)

**Purpose**: Setting up the core development environments and tools that are prerequisites for all modules.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Develop Dockerfile for ROS 2 (Humble/Iron) and Python 3.x development environment `environments/Dockerfile.ros2`
- [x] T008 Develop Dockerfile for NVIDIA Isaac Sim/ROS environment `environments/Dockerfile.isaac`
- [x] T009 Document installation and setup guide for Digital Twin Workstation (Ubuntu 22.04, GPU drivers) `book/docs/setup_guides/digital_twin_workstation.md`
- [x] T010 Document installation and setup guide for Physical AI Edge Kit (Jetson Orin Nano, RealSense camera, USB IMU, microphone) `book/docs/setup_guides/edge_kit.md`
- [x] T011 [P] Create a common `README.md` for `code_examples/` with instructions on how to use Docker environments `code_examples/README.md`
- [x] T012 [P] Create a common `README.md` for `environments/` to explain Docker setup `environments/README.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Complete Physical AI & Humanoid Robotics Course (Priority: P1) 🎯 MVP

**Goal**: Students successfully progress through all course modules, apply concepts in simulations, and complete the capstone project, demonstrating a comprehensive understanding of Physical AI and humanoid robotics.

**Independent Test**: Can be fully tested by a student completing all course modules, exercises, and the capstone project, and demonstrating proficiency in the learning outcomes.

### Module 1: Robotic Nervous System (ROS 2)

- [x] T013 [US1] Create Docusaurus chapter for Module 1: Robotic Nervous System (ROS 2) `book/docs/module1_ros2/index.md`
- [x] T014 [US1] Write content for ROS 2 nodes, topics, and services in `book/docs/module1_ros2/ros2_basics.md`
- [x] T015 [US1] Write content for Python integration with ROS (rclpy) in `book/docs/module1_ros2/python_ros.md`
- [x] T016 [US1] Write content for URDF for humanoids in `book/docs/module1_ros2/urdf_humanoids.md`
- [ ] T017 [US1] Create ROS 2 package `my_robot_pkg` in `code_examples/module1_ros2/my_robot_pkg/`
- [ ] T018 [US1] Implement a basic ROS 2 publisher/subscriber example in `code_examples/module1_ros2/my_robot_pkg/src/simple_publisher.py` and `code_examples/module1_ros2/my_robot_pkg/src/simple_subscriber.py`
- [ ] T019 [US1] Create a basic URDF for a simple robotic arm in `code_examples/module1_ros2/my_robot_pkg/urdf/simple_arm.urdf`

### Module 2: Digital Twin (Gazebo & Unity)

- [x] T020 [US1] Create Docusaurus chapter for Module 2: Digital Twin (Gazebo & Unity) `book/docs/module2_digital_twin/index.md`
- [x] T021 [US1] Write content for physics simulation (gravity, collisions) in `book/docs/module2_digital_twin/physics_sim.md`
- [ ] T022 [US1] Write content for sensor simulation (LiDAR, Depth Cameras, IMUs) in `book/docs/module2_digital_twin/sensor_sim.md`
- [ ] T023 [US1] Write content for visualization in Unity in `book/docs/module2_digital_twin/unity_viz.md`
- [ ] T024 [US1] Create Gazebo world `simple_env.world` with basic physics and objects in `code_examples/module2_simulations/gazebo/worlds/simple_env.world`
- [ ] T025 [US1] Integrate `simple_arm.urdf` into Gazebo simulation `code_examples/module2_simulations/gazebo/models/simple_arm/model.urdf`
- [ ] T026 [US1] Develop a Unity project for robot visualization and interaction `code_examples/module2_simulations/unity/RobotVizProject/`
- [ ] T027 [US1] Implement sensor data visualization (e.g., LiDAR point cloud) in Unity `code_examples/module2_simulations/unity/RobotVizProject/Assets/Scripts/LidarVisualizer.cs`

### Module 3: AI-Robot Brain (NVIDIA Isaac)

- [x] T028 [US1] Create Docusaurus chapter for Module 3: AI-Robot Brain (NVIDIA Isaac) `book/docs/module3_ai_robot_brain/index.md`
- [x] T029 [US1] Write content for Isaac Sim and Isaac ROS introduction `book/docs/module3_ai_robot_brain/isaac_intro.md`
- [x] T030 [US1] Write content for VSLAM concepts and implementation using Isaac ROS `book/docs/module3_ai_robot_brain/vslam.md`
- [x] T031 [US1] Write content for path planning and navigation using Isaac ROS `book/docs/module3_ai_robot_brain/path_planning.md`
- [x] T032 [US1] Write content for reinforcement learning in robotics with Isaac Sim `book/docs/module3_ai_robot_brain/reinforcement_learning.md`
- [x] T033 [US1] Develop a basic VSLAM pipeline using Isaac ROS in `code_examples/module3_isaac_ai/vslam_pipeline/`
- [x] T034 [US1] Implement a simple path planning example in Isaac Sim `code_examples/module3_isaac_ai/path_planning_isaac/`

### Module 4: Vision-Language-Action (VLA)

- [x] T035 [US1] Create Docusaurus chapter for Module 4: Vision-Language-Action (VLA) `book/docs/module4_vla/index.md`
- [x] T036 [US1] Write content for voice-to-action with OpenAI Whisper `book/docs/module4_vla/whisper_integration.md`
- [x] T037 [US1] Write content for natural language planning to ROS 2 actions `book/docs/module4_vla/nlp_to_ros.md`
- [ ] T038 [US1] Implement a basic Whisper integration to transcribe voice commands to text `code_examples/module4_vla/whisper_example/whisper_node.py`
- [ ] T039 [US1] Develop a natural language understanding (NLU) component to parse text commands into ROS 2 actions `code_examples/module4_vla/nlp_commander/nlp_node.py`

### Capstone: Autonomous Humanoid Completes Tasks

- [ ] T040 [US1] Create Docusaurus chapter for Capstone Project `book/docs/capstone_project/index.md`
- [ ] T041 [US1] Write capstone project overview and requirements `book/docs/capstone_project/project_overview.md`
- [ ] T042 [US1] Integrate ROS 2, Gazebo, Isaac perception, and VLA components into a single capstone project `code_examples/capstone_project/`
- [ ] T043 [US1] Develop a high-level task planner for the humanoid robot `code_examples/capstone_project/task_planner.py`
- [ ] T044 [US1] Implement a scenario where the simulated humanoid completes a real-world task based on verbal commands `code_examples/capstone_project/run_scenario.py`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Enhancements, documentation, and integration of optional features.

- [ ] T045 [P] Add course introduction and conclusion chapters to Docusaurus `book/docs/intro.md`, `book/docs/conclusion.md`
- [ ] T046 [P] Implement Docusaurus search functionality `book/docusaurus.config.js`
- [ ] T047 Review and edit all Docusaurus content for clarity, accuracy, and consistency `book/docs/**/*.md`
- [ ] T048 [P] Research and integrate optional personalization features (e.g., student progress tracking) `book/src/theme/` (or similar)
- [ ] T049 [P] Research and integrate optional multi-language content support for Docusaurus `book/docusaurus.config.js`
- [ ] T050 Develop a basic AI-driven RAG chatbot integration for course Q&A `chatbot_integration/`
- [ ] T051 Document hardware requirements and robot lab options in a dedicated Docusaurus section `book/docs/hardware_options.md`
- [ ] T052 Document cloud-native deployment options (AWS / NVIDIA Omniverse) `book/docs/cloud_deployment.md`
- [ ] T053 Run and verify all code examples in `code_examples/` `code_examples/**/*.py` (or similar)
- [ ] T054 Deploy Docusaurus book to GitHub Pages `book/`
- [ ] T055 Create `feature_description.json` with course overview for AI agents `feature_description.json`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Content creation for Docusaurus modules should precede code example implementation where relevant.
- Models/URDF before services/simulations.
- Core implementation before integration.

### Parallel Opportunities

- Many Docusaurus content creation tasks are parallelizable within and across modules.
- Development of Dockerfiles can be parallelized.
- Research for optional features can be parallelized.
- Specific code examples within modules can often be developed in parallel, especially if they have minimal interdependencies.

---

## Parallel Example: User Story 1

```bash
# Example of parallel Docusaurus content creation for Module 1:
Task: "Create Docusaurus chapter for Module 1: Robotic Nervous System (ROS 2) book/docs/module1_ros2/index.md"
Task: "Write content for ROS 2 nodes, topics, and services in book/docs/module1_ros2/ros2_basics.md"
Task: "Write content for Python integration with ROS (rclpy) in book/docs/module1_ros2/python_ros.md"

# Example of parallel code examples creation for Module 1:
Task: "Create ROS 2 package my_robot_pkg in code_examples/module1_ros2/my_robot_pkg/"
Task: "Implement a basic ROS 2 publisher/subscriber example in code_examples/module1_ros2/my_robot_pkg/src/simple_publisher.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (all modules and capstone)
4. **STOP and VALIDATE**: Test User Story 1 independently (through capstone project execution)
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Complete Module 1 (ROS 2) → Test independently → Deploy/Demo
3. Complete Module 2 (Digital Twin) → Test independently → Deploy/Demo
4. Complete Module 3 (AI-Robot Brain) → Test independently → Deploy/Demo
5. Complete Module 4 (VLA) → Test independently → Deploy/Demo
6. Complete Capstone Project → Test independently → Deploy/Demo
7. Each module adds value towards the complete course without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together.
2. Once Foundational is done:
   - Developer A: Module 1 & 2 content and code examples.
   - Developer B: Module 3 & 4 content and code examples.
   - Developer C: Capstone Project integration and Final Polish tasks.
3. Modules complete and integrate independently.

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story/module should be independently completable and testable
- Verify tests fail before implementing (if tests are added)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
