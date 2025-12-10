# Data Model: Physical AI & Humanoid Robotics Educational Book

## Core Entities

### EducationalContent
- **Fields:**
  - id: string (unique identifier for the content piece)
  - title: string (title of the chapter/module)
  - description: string (brief description of the content)
  - category: string (e.g., "setup", "ros2-fundamentals", "simulation", "isaac-platform", "vla", "capstone")
  - learningObjectives: array of strings (3-5 learning objectives)
  - prerequisites: array of strings (what reader needs before starting)
  - content: string (the main content in markdown format)
  - examples: array of strings (file paths to related code examples)
  - diagrams: array of strings (file paths to related diagrams/images)
  - testingVerification: string (how to confirm it works)
  - commonIssues: array of strings (top 3-5 errors with solutions)
  - keyTakeaways: array of strings (summary of key points)
  - nextSteps: string (preview of next chapter)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### CodeExample
- **Fields:**
  - id: string (unique identifier for the example)
  - title: string (descriptive name of the example)
  - description: string (what this example demonstrates)
  - filePath: string (path to the actual code file)
  - language: string (programming language, e.g., "python", "bash", "xml")
  - prerequisites: array of strings (packages/setup needed)
  - expectedOutput: string (what success looks like)
  - imports: array of strings (import statements with explanations)
  - mainComponents: array of strings (key function/class definitions)
  - errorHandling: string (how errors are handled)
  - usageInstructions: string (how to run/execute the example)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### URDFModel
- **Fields:**
  - id: string (unique identifier for the robot model)
  - name: string (name of the robot)
  - description: string (description of the robot and its capabilities)
  - filePath: string (path to the URDF file)
  - links: array of objects (physical links of the robot)
  - joints: array of objects (joints connecting the links)
  - sensors: array of objects (sensor definitions)
  - actuators: array of objects (actuator definitions)
  - materials: array of objects (material definitions)
  - gazeboPlugins: array of strings (Gazebo plugin configurations)
  - validationStatus: string (e.g., "valid", "needs_fixes", "invalid")
  - compatibility: array of strings (ROS distributions it works with)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### SimulationEnvironment
- **Fields:**
  - id: string (unique identifier for the environment)
  - name: string (name of the environment/world)
  - description: string (description of the environment)
  - filePath: string (path to the world file)
  - platform: string (e.g., "gazebo", "isaac-sim", "unity")
  - objects: array of objects (static and dynamic objects in the environment)
  - lighting: object (lighting configuration)
  - physicsProperties: object (friction, gravity, etc.)
  - robotSpawnPoints: array of objects (where robots can be spawned)
  - testingScenarios: array of strings (specific tests that can be run)
  - performanceMetrics: object (FPS, simulation speed, etc.)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### HardwareSpecification
- **Fields:**
  - id: string (unique identifier for the hardware config)
  - name: string (name of the configuration, e.g., "Budget Tier", "Standard Tier")
  - category: string (e.g., "robot", "computer", "sensors", "accessories")
  - components: array of objects (individual hardware components)
  - totalCost: number (estimated total cost)
  - currency: string (currency code, e.g., "USD")
  - lastUpdated: date (when prices were last verified)
  - availability: string (current availability status)
  - compatibility: array of strings (systems it's compatible with)
  - alternatives: array of objects (alternative components with trade-offs)
  - pros: array of strings (advantages of this configuration)
  - cons: array of strings (disadvantages of this configuration)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### LearningAssessment
- **Fields:**
  - id: string (unique identifier for the assessment)
  - title: string (title of the assessment)
  - category: string (e.g., "quiz", "hands-on", "project", "capstone")
  - targetChapter: string (which chapter/module this assesses)
  - objectives: array of strings (learning objectives being assessed)
  - questions: array of objects (questions in the assessment)
  - rubric: object (grading criteria and point values)
  - expectedDuration: number (estimated time to complete in minutes)
  - difficulty: string (e.g., "beginner", "intermediate", "advanced")
  - passingCriteria: object (what constitutes passing)
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

### TroubleshootingGuide
- **Fields:**
  - id: string (unique identifier for the guide entry)
  - category: string (e.g., "setup", "simulation", "hardware", "network")
  - problemDescription: string (detailed description of the problem)
  - errorMessages: array of strings (common error messages associated with the problem)
  - rootCause: string (likely cause of the problem)
  - solutions: array of objects (step-by-step solutions)
  - prevention: string (how to prevent the issue in the future)
  - verification: string (how to verify the solution worked)
  - frequency: string (how often this issue occurs, e.g., "common", "rare")
  - createdAt: date (creation timestamp)
  - updatedAt: date (last modification timestamp)

## Relationships

### EducationalContent contains CodeExample
- One EducationalContent can reference multiple CodeExample
- Each CodeExample belongs to one or more EducationalContent

### EducationalContent uses URDFModel
- One EducationalContent can reference multiple URDFModel
- Each URDFModel can be used by multiple EducationalContent

### EducationalContent uses SimulationEnvironment
- One EducationalContent can reference multiple SimulationEnvironment
- Each SimulationEnvironment can be used by multiple EducationalContent

### EducationalContent includes TroubleshootingGuide
- One EducationalContent can reference multiple TroubleshootingGuide
- Each TroubleshootingGuide can be relevant to multiple EducationalContent

### HardwareSpecification referenced by EducationalContent
- EducationalContent may reference specific HardwareSpecification
- HardwareSpecification may be referenced by multiple EducationalContent

### LearningAssessment targets EducationalContent
- One LearningAssessment assesses one or more EducationalContent
- One EducationalContent may have multiple LearningAssessment

## Validation Rules

### EducationalContent
- title must be 5-100 characters
- learningObjectives must have 3-5 items
- content must follow the chapter template structure
- examples must exist as valid files in the examples directory
- createdAt and updatedAt must be valid ISO date strings

### CodeExample
- filePath must exist in the repository
- language must be a supported language (python, bash, xml, etc.)
- expectedOutput must be non-empty
- createdAt and updatedAt must be valid ISO date strings

### URDFModel
- filePath must point to a valid URDF file
- validationStatus must be one of: "valid", "needs_fixes", "invalid"
- compatibility must reference valid ROS distributions
- createdAt and updatedAt must be valid ISO date strings

### SimulationEnvironment
- filePath must point to a valid world file
- platform must be one of: "gazebo", "isaac-sim", "unity"
- performanceMetrics must include FPS and simulation speed
- createdAt and updatedAt must be valid ISO date strings

### HardwareSpecification
- totalCost must be a positive number
- currency must be a valid currency code
- lastUpdated must be within 30 days for current pricing
- createdAt and updatedAt must be valid ISO date strings

### LearningAssessment
- targetChapter must reference an existing EducationalContent
- passingCriteria must be clearly defined
- expectedDuration must be a positive number
- createdAt and updatedAt must be valid ISO date strings

### TroubleshootingGuide
- frequency must be one of: "common", "moderate", "rare"
- createdAt and updatedAt must be valid ISO date strings