---
id: 1
title: "RAG Chatbot Creation"
stage: spec
date_iso: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 001-rag-chatbot
user: Admin
command: /sp.specify
labels:
  - rag
  - chatbot
  - textbook
  - ai
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/001-rag-chatbot/spec.md"
  - "specs/001-rag-chatbot/checklists/requirements.md"
tests_yaml:
  - "Specification quality checklist completed"
prompt_text: |
  /sp.specify

  You are a **Senior AI Engineer and Retrieval-Augmented Generation (RAG) Architect**
  working on the project:

  **"Physical AI & Humanoid Robotics – AI-Native Textbook"**

  This textbook is written using **Docusaurus + Spec-Kit Plus** and contains:
  - Module 1: ROS 2 Nervous System
  - Module 2: Digital Twin (Gazebo & Unity)
  - Module 3: NVIDIA Isaac (AI-Robot Brain)
  - Module 4: Vision-Language-Action (VLA)
  - Hardware Architecture & Lab Design
  - Weekly Breakdown & Learning Outcomes
  module 5:Rag chatbot

  ---

  ## OBJECTIVE
  Design and implement an **Integrated RAG Chatbot** that allows readers to:

  1. Ask questions about the **entire book**
  2. Ask questions based on **user-selected text only**
  3. Learn Physical AI concepts interactively using an AI Agent

  The chatbot must satisfy **Panaversity Hackathon Requirements**.

  ---

  ## DATA SOURCE
  - Primary source: `/docs` directory of the Docusaurus book
  - Content format: Markdown (`.md` / `.mdx`)
  - Treat each **section and subsection** as a retrievable knowledge chunk

  ---

  ## RAG ARCHITECTURE REQUIREMENTS

  ### 1. Chunking Strategy
  - Chunk by:
    - Module
    - Chapter
    - Subsection
  - Maintain metadata:
    - `module`
    - `chapter`
    - `section_title`
    - `source_file`

  ---

  ### 2. Embeddings
  - Use OpenAI Embeddings
  - Store vectors in **Qdrant Cloud (Free Tier)**
  - Store metadata + chat history in **Neon Serverless Postgres**

  ---

  ### 3. Retrieval Logic
  - Default Mode: Retrieve relevant chunks from the **entire book**
  - Selected-Text Mode:
    - Restrict retrieval strictly to user-highlighted text
    - Do NOT use external or unrelated chapters

  ---

  ### 4. Agent Behavior (CRITICAL)
  The AI Agent must:
  - Answer **only from retrieved book content**
  - Say clearly:
    _"This answer is based on the Physical AI & Humanoid Robotics textbook."_
  - Avoid hallucination
  - Use beginner-friendly explanations
  - Provide technical depth when needed

  ---

  ### 5. Technologies (MANDATORY)
  - OpenAI Agents / ChatKit SDK
  - FastAPI backend
  - Qdrant Cloud (Vector DB)
  - Neon Serverless Postgres
  - Embedded inside the Docusaurus site

  ---

  ## USER EXPERIENCE
  - Floating chatbot UI inside the book
  - Input modes:
    - "Ask the Book"
    - "Ask Selected Text"
  - Clear loading + citation indication
  - Educational
response_text: "Created comprehensive specification for the RAG Chatbot feature, including user scenarios, functional requirements, and success criteria. The spec was validated against quality criteria and implementation details were removed to maintain technology-agnostic approach."
outcome: "Specification created and validated successfully"
evaluation: "The specification meets all requirements and is ready for the planning phase"
---

# Prompt History Record

## Summary
Created comprehensive specification for the RAG Chatbot feature for the Physical AI & Humanoid Robotics textbook.

## Details
- **ID**: 1
- **Title**: RAG Chatbot Creation
- **Stage**: spec
- **Date**: 2025-12-15
- **Surface**: agent
- **Model**: Claude Sonnet 4.5
- **Feature**: rag-chatbot
- **Branch**: 001-rag-chatbot
- **User**: Admin
- **Command**: /sp.specify

## Files Modified
- specs/001-rag-chatbot/spec.md
- specs/001-rag-chatbot/checklists/requirements.md

## Tests Performed
- Specification quality checklist completed