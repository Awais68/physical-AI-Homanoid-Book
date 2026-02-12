# Quickstart Guide: Physical AI Edge Kit with RAG Chatbot

## Prerequisites

- Docker and Docker Compose installed
- Linux-based edge computing hardware or development machine
- Physical AI device or simulation environment available
- OpenAI API key for RAG functionality
- Qdrant Cloud account and API key
- Neon Postgres database setup
- Network connectivity (for initial setup and RAG features)

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/physical-ai-edge-kit.git
   cd physical-ai-edge-kit
   ```

2. Navigate to the edge deployment directory:
   ```bash
   cd deployment/edge
   ```

3. Configure environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your specific configuration including:
   # - OpenAI API key
   # - Qdrant Cloud URL and API key
   # - Neon Postgres connection details
   # - Better-Auth configuration
   ```

## Deployment

1. Start the edge services:
   ```bash
   docker-compose up -d
   ```

2. Verify all services are running:
   ```bash
   docker-compose ps
   ```

3. Initialize the databases:
   ```bash
   docker-compose exec backend python manage.py init-db
   docker-compose exec backend python manage.py init-rag
   ```

## Setting up RAG Chatbot

1. Index your educational content:
   ```bash
   docker-compose exec backend python -m src.rag.index_documents --source-path /path/to/educational/content
   ```

2. Verify the knowledge base is populated:
   ```bash
   docker-compose exec backend python -m src.rag.check_index
   ```

## Connecting Physical AI Devices

1. Ensure your physical AI device is powered on and connected to the same network
2. Register the device using the API:
   ```bash
   curl -X POST http://localhost:8000/api/devices \
        -H "Content-Type: application/json" \
        -d '{
          "id": "robot-001",
          "name": "Educational Robot 1",
          "type": "humanoid",
          "connectionInfo": {
            "ip": "192.168.1.100",
            "port": 8888
          },
          "safetyParameters": {
            "maxSpeed": 0.5,
            "operationalBoundary": "safe_zone"
          }
        }'
   ```

## Starting a Learning Session

1. Create a new learning session:
   ```bash
   curl -X POST http://localhost:8000/api/sessions \
        -H "Content-Type: application/json" \
        -d '{
          "participants": ["student-001", "student-002"],
          "educatorId": "educator-001",
          "deviceIds": ["robot-001"],
          "activities": ["basic_movement", "safety_demo"]
        }'
   ```

2. Monitor the session through the web dashboard at `http://localhost:3000`

## Using the RAG Chatbot

1. Access the chatbot through the web interface or API
2. Ask questions about robotics, the Physical AI Edge Kit, or educational content
3. The chatbot will provide answers based on indexed documentation and educational materials
4. For text-specific queries, select text in the UI and use the "Ask Chatbot" option

## Safety Monitoring

The system continuously monitors for safety events. You can check the current safety status:

```bash
curl http://localhost:8000/api/safety/monitor
```

## Authentication and Personalization

1. Users can register/login using the better-auth system
2. Personalized features include:
   - Bookmarks for educational content
   - Learning progress tracking
   - Personalized recommendations
   - Language preferences

## Multilingual Support

1. The system supports multiple languages
2. Users can switch languages in their preferences
3. Content and interface elements will be translated accordingly

## Simulation Mode

For testing without physical hardware, you can run the simulation:

```bash
docker-compose -f docker-compose.simulation.yml up -d
```

## Troubleshooting

- If services fail to start, check logs with `docker-compose logs`
- Verify network connectivity to physical devices
- Ensure safety parameters are properly configured
- Check that the database is accessible
- Verify API keys for OpenAI, Qdrant, and Neon Postgres are correct
- Check that the RAG indexing process completed successfully

## Next Steps

- Review the full API documentation
- Configure your specific educational platform integration
- Set up user accounts for educators and students
- Customize safety parameters for your environment
- Add your educational content to the RAG knowledge base