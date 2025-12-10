# Data Model: AI-Native Textbook — Physical AI & Humanoid Robotics

## Core Entities

### Textbook Module
**Description**: Represents one of the four main curriculum sections containing multiple weeks of content
- **Fields**:
  - id: string (unique identifier, e.g., "module-1-robotic-nervous-system")
  - title: string (display title, e.g., "The Robotic Nervous System")
  - description: string (brief description of the module)
  - order: integer (sequence number 1-4)
  - weeks: array of Week entities (associated weeks in the module)
  - createdAt: timestamp
  - updatedAt: timestamp

**Validation Rules**:
- id must be unique across all modules
- order must be between 1 and 4
- weeks array must contain 2-4 week entities (depending on module)

### Curriculum Week
**Description**: Represents a single week of focused learning content within a module
- **Fields**:
  - id: string (unique identifier, e.g., "week-1-introduction-to-physical-ai")
  - title: string (display title, e.g., "Introduction to Physical AI and Sensors")
  - module: Module entity (parent module)
  - order: integer (sequence number within the module)
  - content: string (Markdown content for the week)
  - prerequisites: array of string (prerequisites for this week)
  - objectives: array of string (learning objectives)
  - createdAt: timestamp
  - updatedAt: timestamp

**Validation Rules**:
- id must be unique across all weeks
- order must be sequential within the module (1, 2, 3, etc.)
- content must be valid Markdown
- module reference must exist

### Docusaurus Document
**Description**: Represents a Docusaurus-compatible document file containing textbook content
- **Fields**:
  - id: string (unique identifier, same as week id)
  - path: string (file path relative to docs/ directory)
  - title: string (document title)
  - sidebar_label: string (label for sidebar navigation)
  - sidebar_position: integer (position in sidebar)
  - custom_edit_url: string (URL for custom edit link, optional)
  - content: string (full Markdown content with frontmatter)
  - createdAt: timestamp
  - updatedAt: timestamp

**Validation Rules**:
- path must follow Docusaurus conventions (e.g., "module-1/week-1-introduction.md")
- sidebar_position must be unique within the parent category
- content must include proper Docusaurus frontmatter

### Navigation Sidebar
**Description**: Represents the sidebar navigation structure for the textbook
- **Fields**:
  - id: string (unique identifier, e.g., "textbook-sidebar")
  - items: array of SidebarItem entities
  - createdAt: timestamp
  - updatedAt: timestamp

### SidebarItem
**Description**: Represents a single item in the navigation sidebar
- **Fields**:
  - type: string (e.g., "category", "doc")
  - label: string (display label)
  - link: object (for category type, contains type and id)
  - items: array of SidebarItem (for category type, nested items)
  - id: string (for doc type, references Docusaurus document)
  - position: integer (position in sidebar)

## Relationships

### Module to Week
- **Type**: One-to-Many
- **Description**: A module contains multiple weeks of content
- **Cardinality**: 1 Module : N Weeks
- **Constraint**: Each week must belong to exactly one module

### Week to Docusaurus Document
- **Type**: One-to-One
- **Description**: Each week corresponds to a single Docusaurus document
- **Cardinality**: 1 Week : 1 Document
- **Constraint**: Document path must match week's module structure

### Navigation Sidebar to Sidebar Items
- **Type**: One-to-Many
- **Description**: A sidebar contains multiple navigation items
- **Cardinality**: 1 Sidebar : N SidebarItems
- **Constraint**: Items must be properly nested to reflect module/week hierarchy

## State Transitions

### Content Development States
- **Draft**: Content is being created/edited
- **Review**: Content is under review for technical accuracy
- **Published**: Content is ready for deployment
- **Archived**: Content is no longer used (rare for textbook content)

## Validation Rules Summary

1. **Curriculum Structure Validation**:
   - Total modules must equal 4
   - Total weeks must equal 13
   - Module 1: 3 weeks (Weeks 1-3)
   - Module 2: 2 weeks (Weeks 4-5)
   - Module 3: 3 weeks (Weeks 6-8)
   - Module 4: 4 weeks (Weeks 9-13)

2. **Content Validation**:
   - All content must be technically accurate for ROS 2, Gazebo, NVIDIA Isaac, and VLA
   - Content must follow pedagogical progression from basic to advanced concepts
   - Code examples must be valid and executable

3. **Navigation Validation**:
   - Sidebar must reflect the 4-module, 13-week structure
   - Navigation must be intuitive and sequential
   - All documents must be accessible through the sidebar