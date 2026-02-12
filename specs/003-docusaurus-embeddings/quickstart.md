# Quickstart Guide: Docusaurus Embeddings Generation

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant Cloud cluster URL and API key
- Target Docusaurus documentation site URL

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/docusaurus-embeddings.git
   cd docusaurus-embeddings
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your Cohere and Qdrant credentials
   ```

## Configuration

1. Create a configuration file `config/settings.yaml`:
   ```yaml
   cohere:
     api_key: "your-cohere-api-key"
     model: "embed-english-v2.0"

   qdrant:
     url: "https://your-cluster.qdrant.tech"
     api_key: "your-qdrant-api-key"
     collection_name: "docusaurus-docs"

   crawler:
     delay: 1  # seconds between requests
     timeout: 30  # seconds for each request
     max_pages: 1000  # maximum pages to crawl

   chunking:
     max_chunk_size: 1000  # characters per chunk
     overlap: 200  # character overlap between chunks
   ```

## Usage

### Process a Docusaurus Site

1. Run the processing pipeline:
   ```bash
   python -m src.cli.process --url https://your-docusaurus-site.com
   ```

2. Monitor the processing status:
   ```bash
   python -m src.cli.status --job-id <job-id>
   ```

### Advanced Options

- Process with custom configuration:
  ```bash
  python -m src.cli.process --url https://your-site.com --config custom-config.yaml
  ```

- Process only specific sections:
  ```bash
  python -m src.cli.process --url https://your-site.com --include-pattern "/docs/api/*"
  ```

- Skip certain pages:
  ```bash
  python -m src.cli.process --url https://your-site.com --exclude-pattern "/docs/internal/*"
  ```

## Verification

1. Check that embeddings were stored in Qdrant:
   ```bash
   python -m src.cli.verify
   ```

2. Test similarity search:
   ```bash
   python -m src.cli.search --query "your search query"
   ```

## Troubleshooting

- If you get rate limit errors from Cohere, reduce the number of concurrent requests
- If crawling is too slow, adjust the delay between requests in the configuration
- If memory usage is high, process fewer documents per batch
- Check logs in `logs/` directory for detailed error information

## Next Steps

- Integrate with your RAG application
- Set up scheduled processing for documentation updates
- Configure monitoring and alerts for processing jobs