# Data Model: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Entity: ChatSession
**Description**: Represents a user's conversation session with the chatbot
**Fields**:
- id (UUID): Unique identifier for the session
- user_id (UUID, optional): Identifier for authenticated user (null for anonymous sessions)
- created_at (timestamp): Time when session was created
- last_activity_at (timestamp): Time of last interaction
- metadata (JSON): Additional session information (e.g., current page, preferences)

**Validation Rules**:
- id must be unique
- created_at must be before or equal to last_activity_at
- metadata must be valid JSON

**Relationships**:
- One-to-many with Message entities (a session contains multiple messages)

## Entity: Message
**Description**: Represents a single message in a conversation (either user query or system response)
**Fields**:
- id (UUID): Unique identifier for the message
- session_id (UUID): Reference to the parent ChatSession
- role (string): Either "user" or "assistant"
- content (text): The actual message content
- timestamp (timestamp): When the message was created
- context_reference (string, optional): Reference to specific textbook section used in response
- embeddings_metadata (JSON, optional): Information about which text chunks were used for RAG

**Validation Rules**:
- session_id must reference an existing ChatSession
- role must be either "user" or "assistant"
- content must not be empty
- timestamp must be after session creation

**Relationships**:
- Many-to-one with ChatSession (message belongs to one session)

## Entity: TextbookContent
**Description**: Represents processed textbook content chunks for RAG retrieval
**Fields**:
- id (UUID): Unique identifier for the content chunk
- chapter_id (string): Identifier for the textbook chapter
- section_title (string): Title of the section this chunk belongs to
- content_text (text): The actual text content of this chunk
- embedding_vector (JSON): Vector representation for similarity search
- page_reference (string): Reference to the original page/section
- created_at (timestamp): When this chunk was processed

**Validation Rules**:
- id must be unique
- content_text must not be empty
- embedding_vector must be a valid vector representation

## Entity: User (Future Implementation)
**Description**: Represents a registered user (for future authentication features)
**Fields**:
- id (UUID): Unique identifier for the user
- email (string): User's email address
- created_at (timestamp): Account creation time
- preferences (JSON): User preferences for the chatbot

**Validation Rules**:
- email must be unique and valid
- id must be unique

**Note**: This entity is for future implementation when user authentication is added.

## State Transitions

### ChatSession
- **Active**: When a user starts a new session
- **Inactive**: When the session hasn't been used for a certain period (e.g., 30 minutes)
- **Archived**: When the session is old and can be cleaned up (e.g., after 30 days)

### Message
- **Pending**: When a user submits a query (temporary state)
- **Processed**: When the chatbot has generated a response
- **Delivered**: When the response is shown to the user