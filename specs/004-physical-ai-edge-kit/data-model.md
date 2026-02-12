# Data Model: Physical AI Edge Kit with RAG Chatbot for Educational Robotics

## EdgeDevice
Represents a physical AI device or humanoid robot connected to the edge kit.

**Fields:**
- id: string (unique identifier)
- name: string (display name for the device)
- type: string (robot type/model)
- status: string (online, offline, error, maintenance)
- connectionInfo: object (IP address, port, communication protocol)
- safetyParameters: object (speed limits, operational boundaries, emergency settings)
- lastSeen: datetime (timestamp of last communication)
- capabilities: array (list of device capabilities)

**Validation:**
- id must be unique
- name must not be empty
- status must be one of: online, offline, error, maintenance

## SafetyMonitor
Represents the safety monitoring system that oversees all physical AI interactions.

**Fields:**
- id: string (unique identifier)
- activeRules: array (list of active safety rules)
- alertThresholds: object (thresholds for different types of alerts)
- emergencyProcedures: object (emergency stop procedures and actions)
- monitoringStatus: string (active, paused, error)
- lastAlert: datetime (timestamp of last safety alert)
- safetyMetrics: object (statistics about safety events)

**Validation:**
- id must be unique
- activeRules must be a valid array of safety rules
- monitoringStatus must be one of: active, paused, error

## LearningSession
Represents an educational session involving physical AI interactions.

**Fields:**
- id: string (unique identifier)
- participants: array (list of student IDs participating)
- educatorId: string (ID of supervising educator)
- deviceIds: array (IDs of devices used in the session)
- startTime: datetime (when the session started)
- endTime: datetime (when the session ended, null if ongoing)
- activities: array (list of activities performed during the session)
- safetyData: object (safety events and metrics during the session)
- learningOutcomes: object (assessment data and learning metrics)

**Validation:**
- id must be unique
- participants must be a valid array of student IDs
- startTime must be before endTime if session is completed

## EducatorDashboard
Represents the administrative interface for educators.

**Fields:**
- id: string (unique identifier)
- educatorId: string (ID of the educator)
- monitoredSessions: array (IDs of sessions being monitored)
- safetyControls: object (current safety control settings)
- monitoringPreferences: object (dashboard configuration preferences)
- alerts: array (list of recent alerts)
- reportSettings: object (settings for generating reports)

**Validation:**
- id must be unique
- educatorId must be valid

## StudentProfile
Represents student information relevant to physical AI interactions.

**Fields:**
- id: string (unique identifier)
- name: string (student name)
- permissions: array (list of permissions and access levels)
- safetyTraining: object (safety training completion status)
- learningProgress: object (progress in robotics education)
- interactionHistory: array (history of interactions with physical AI systems)

**Validation:**
- id must be unique
- safetyTraining must be a valid object with required fields

## ChatSession
Represents a conversation session with the RAG chatbot.

**Fields:**
- id: string (unique identifier)
- userId: string (ID of user having the conversation)
- conversationHistory: array (list of message objects with role and content)
- context: object (current context for the conversation)
- sourceCitations: array (list of source documents referenced in responses)
- createdAt: datetime (when the session was created)
- updatedAt: datetime (when the session was last updated)

**Validation:**
- id must be unique
- userId must reference an existing user
- conversationHistory must be an array of valid message objects

## KnowledgeDocument
Represents indexed content from documentation and educational materials used by the RAG system.

**Fields:**
- id: string (unique identifier)
- content: string (the actual text content)
- metadata: object (information about the source, type, etc.)
- sourceUrl: string (URL or path to the original document)
- title: string (title of the document)
- tags: array (list of tags for categorization)
- embeddingVector: array (vector representation for similarity search)
- createdAt: datetime (when the document was indexed)
- updatedAt: datetime (when the document was last updated)

**Validation:**
- id must be unique
- content must not be empty
- embeddingVector must have the expected dimensions for the model

## UserPreference
Represents user-specific settings and preferences.

**Fields:**
- id: string (unique identifier)
- userId: string (ID of the user these preferences belong to)
- language: string (preferred language code)
- personalizationSettings: object (settings for personalization features)
- interfaceConfig: object (UI configuration preferences)
- createdAt: datetime (when preferences were first set)
- updatedAt: datetime (when preferences were last updated)

**Validation:**
- id must be unique
- userId must reference an existing user

## Bookmark
Represents user-saved content references.

**Fields:**
- id: string (unique identifier)
- userId: string (ID of the user who created the bookmark)
- contentReference: string (reference to the content being bookmarked)
- title: string (title of the bookmark)
- notes: string (optional user notes)
- location: object (location information in the content)
- createdAt: datetime (when the bookmark was created)
- updatedAt: datetime (when the bookmark was last updated)

**Validation:**
- id must be unique
- userId must reference an existing user

## UserLearningProgress
Represents tracked learning progress for users.

**Fields:**
- id: string (unique identifier)
- userId: string (ID of the user)
- completedModules: array (list of completed educational modules)
- achievements: array (list of earned achievements)
- personalizedRecommendations: array (recommended content based on progress)
- progressMetrics: object (quantitative metrics about learning progress)
- lastActivity: datetime (timestamp of last learning activity)

**Validation:**
- id must be unique
- userId must reference an existing user

## TranslationResource
Represents localized content and interface translations.

**Fields:**
- id: string (unique identifier)
- languageCode: string (language code for the translation)
- resourceType: string (type of resource: 'content', 'interface', 'documentation')
- resourceId: string (ID of the original resource being translated)
- translatedContent: string (the translated content)
- context: object (context information for proper translation)
- updatedAt: datetime (when the translation was last updated)

**Validation:**
- id must be unique
- languageCode must be a valid language code
- resourceId must reference an existing resource

## Relationships

- One SafetyMonitor can monitor multiple EdgeDevices
- One LearningSession can involve multiple EdgeDevices and multiple Students
- One Educator can supervise multiple LearningSessions
- Multiple Students can participate in one LearningSession
- One Educator has one EducatorDashboard
- One User can have multiple ChatSessions
- Multiple KnowledgeDocuments can be referenced in one ChatSession
- One User can have multiple Bookmarks
- One User has one UserLearningProgress
- One User has one UserPreference
- One TranslationResource maps to one original resource in a specific language