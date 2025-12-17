---
description: "Task list for RAG Chatbot implementation"
---

# Tasks: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `book/src/components/`
- **Configuration**: `backend/src/config/settings.py`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend project structure in backend/
- [X] T002 Initialize Python 3.11 project with FastAPI, OpenAI SDK, Qdrant client, Neon Postgres client dependencies
- [X] T003 [P] Configure linting and formatting tools (black, flake8, mypy) in backend/
- [X] T004 Create frontend component structure in book/src/components/RagChatbot/
- [X] T005 Create environment configuration file backend/.env.example

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Setup database connection framework and configuration in backend/src/config/database.py
- [X] T007 [P] Setup Qdrant vector database connection in backend/src/config/vector_db.py
- [X] T008 [P] Setup API routing and middleware structure in backend/src/main.py
- [X] T009 Create base models/entities that all stories depend on in backend/src/models/
- [X] T010 Configure error handling and logging infrastructure in backend/src/utils/
- [X] T011 Setup environment configuration management in backend/src/config/settings.py
- [X] T012 Create textbook content processing script in backend/src/scripts/process_textbook.py
- [X] T013 [P] Create basic health check endpoint in backend/src/api/health_endpoints.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Entire Book (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask questions about any topic in the textbook and receive answers based only on textbook content with proper citations

**Independent Test**: Can ask questions about various topics across the textbook modules and verify that the AI provides accurate, textbook-based answers with proper citations

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T014 [P] [US1] Contract test for POST /api/v1/chat/query endpoint in backend/tests/contract/test_chat_api.py
- [ ] T015 [P] [US1] Integration test for "Ask the Book" mode in backend/tests/integration/test_rag_chat.py

### Implementation for User Story 1

- [X] T016 [P] [US1] Create ChatSession model in backend/src/models/chat_session.py
- [X] T017 [P] [US1] Create UserQuery model in backend/src/models/user_query.py
- [X] T018 [P] [US1] Create TextbookChunk model in backend/src/models/textbook_chunk.py
- [X] T019 [P] [US1] Create RetrievedContext model in backend/src/models/retrieved_context.py
- [X] T020 [P] [US1] Create ChatResponse model in backend/src/models/chat_response.py
- [X] T021 [US1] Implement RAGService in backend/src/services/rag_service.py
- [X] T022 [US1] Implement VectorStoreService in backend/src/services/vector_store_service.py
- [X] T023 [US1] Implement ChatHistoryService in backend/src/services/chat_history_service.py
- [X] T024 [US1] Implement POST /api/v1/chat/query endpoint in backend/src/api/chat_endpoints.py
- [X] T025 [US1] Add validation for query_mode enum and required fields
- [X] T026 [US1] Add logging for user story 1 operations
- [X] T027 [US1] Create frontend Chatbot component in book/src/components/RagChatbot/Chatbot.jsx
- [X] T028 [US1] Create frontend ChatInterface component in book/src/components/RagChatbot/ChatInterface.jsx
- [X] T029 [US1] Integrate chatbot UI with backend API in book/src/components/RagChatbot/Chatbot.jsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask Questions About Selected Text Only (Priority: P2)

**Goal**: Enable students to ask questions about only the text they have highlighted/selected, with AI responding using only that selected text content

**Independent Test**: Can select text on a page, ask questions about it, and verify that the AI only uses that selected text to form responses

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T030 [P] [US2] Contract test for selected text query functionality in backend/tests/contract/test_chat_api.py
- [ ] T031 [P] [US2] Integration test for "Ask Selected Text" mode in backend/tests/integration/test_rag_chat.py

### Implementation for User Story 2

- [X] T032 [P] [US2] Extend UserQuery model to handle selected_text in backend/src/models/user_query.py
- [X] T033 [US2] Update RAGService to support selected text mode in backend/src/services/rag_service.py
- [X] T034 [US2] Implement text selection handling in backend/src/services/rag_service.py
- [X] T035 [US2] Update chat endpoint to support selected text mode in backend/src/api/chat_endpoints.py
- [X] T036 [US2] Create TextSelectionHandler component in book/src/components/RagChatbot/TextSelectionHandler.jsx
- [X] T037 [US2] Integrate text selection with chat functionality in book/src/components/RagChatbot/Chatbot.jsx
- [X] T038 [US2] Add UI for "Ask Selected Text" mode in book/src/components/RagChatbot/ChatInterface.jsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Interactive Learning with AI Agent (Priority: P3)

**Goal**: Enable students to engage in interactive conversations with an AI agent that explains concepts in beginner-friendly terms while providing technical depth when needed

**Independent Test**: Can engage in multi-turn conversations where the AI adjusts explanation complexity based on user queries and maintains context across the conversation

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T039 [P] [US3] Contract test for conversation context maintenance in backend/tests/contract/test_chat_api.py
- [ ] T040 [P] [US3] Integration test for explanation complexity adjustment in backend/tests/integration/test_rag_chat.py

### Implementation for User Story 3

- [X] T041 [P] [US3] Update ChatSession model to support complexity preferences in backend/src/models/chat_session.py
- [X] T042 [US3] Implement explanation complexity adjustment in backend/src/services/rag_service.py
- [X] T043 [US3] Update chat endpoint to support explanation preferences in backend/src/api/chat_endpoints.py
- [X] T044 [US3] Add conversation context maintenance in backend/src/services/chat_history_service.py
- [X] T045 [US3] Create UI controls for explanation complexity in book/src/components/RagChatbot/ChatInterface.jsx
- [X] T046 [US3] Update chatbot to support multi-turn conversations in book/src/components/RagChatbot/Chatbot.jsx

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Additional Features (Priority: P4)

**Goal**: Implement additional features based on functional requirements (FR-007, FR-008, FR-009)

**Independent Test**: Can retrieve chat history, see loading indicators, and view proper citations

### Implementation for Additional Features

- [X] T047 [P] Implement GET /chat/session/{session_id} endpoint in backend/src/api/chat_endpoints.py
- [X] T048 [P] Implement POST /chat/session/{session_id}/clear endpoint in backend/src/api/chat_endpoints.py
- [X] T049 [P] Add loading indicators in frontend components in book/src/components/RagChatbot/
- [X] T050 [P] Add citation display in chat responses in book/src/components/RagChatbot/ChatInterface.jsx
- [X] T051 [P] Update response format to include proper citations in backend/src/services/rag_service.py

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T052 [P] Documentation updates in backend/docs/ and book/docs/
- [X] T053 Code cleanup and refactoring
- [X] T054 Performance optimization across all stories
- [X] T055 [P] Additional unit tests (if requested) in backend/tests/unit/
- [X] T056 Security hardening
- [X] T057 Run quickstart.md validation
- [X] T058 Update Docusaurus site to integrate chatbot component in book/src/pages/
- [X] T059 Create README with setup instructions for the RAG chatbot feature

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create ChatSession model in backend/src/models/chat_session.py"
Task: "Create UserQuery model in backend/src/models/user_query.py"
Task: "Create TextbookChunk model in backend/src/models/textbook_chunk.py"
Task: "Create RetrievedContext model in backend/src/models/retrieved_context.py"
Task: "Create ChatResponse model in backend/src/models/chat_response.py"

# Launch all services for User Story 1 together:
Task: "Implement RAGService in backend/src/services/rag_service.py"
Task: "Implement VectorStoreService in backend/src/services/vector_store_service.py"
Task: "Implement ChatHistoryService in backend/src/services/chat_history_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence