from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import set_tracing_disabled, function_tool
import os
from dotenv import load_dotenv
from agents import enable_verbose_stdout_logging
import cohere
from qdrant_client import QdrantClient
import os
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")


# Initialize Cohere client
cohere_client = cohere.Client("8tK6O5xlVMGGgfFY3zG8koo4sTHE1rDSLhR3PUtH")


# Connect to Qdrant
qdrant = QdrantClient(
    url="https://d781f662-2044-4110-a0aa-9f08549ea800.us-east4-0.gcp.cloud.qdrant.io",
    api_key="QDRANT_API_KEY",
    timeout=60,  # Increase timeout
)


enable_verbose_stdout_logging()

load_dotenv()
set_tracing_disabled(disabled=True)

gemini_api_key = os.getenv("GEMINI_API_KEY")
provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",    openai_client=provider
)
# @function_tool
# def retrieve(query):
#     embedding = get_embedding(query)
#     result = qdrant.query_points(
#         collection_name="physical_ai_docs",
#         query=embedding,
#         limit=5
#     )
#     return [point.payload["text"] for point in result.points]




def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


@function_tool
def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="physical_ai_docs",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]



agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
""",
    model=model,
    tools=[retrieve]
)


result = Runner.run_sync(
    agent,
    input="what is physical ai?",
)

print(result.final_output)