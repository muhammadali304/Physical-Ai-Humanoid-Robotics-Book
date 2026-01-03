#!/bin/bash
# Deployment script for the RAG Ingestion Pipeline

set -e  # Exit immediately if a command exits with a non-zero status

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting deployment of RAG Ingestion Pipeline${NC}"

# Check if Docker is installed and running
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Docker is not installed. Please install Docker and try again.${NC}"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo -e "${RED}Docker daemon is not running. Please start Docker and try again.${NC}"
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo -e "${YELLOW}docker-compose is not installed. Installing...${NC}"
    sudo apt-get update
    sudo apt-get install -y docker-compose-plugin || {
        echo -e "${RED}Failed to install docker-compose. Please install it manually and try again.${NC}"
        exit 1
    }
fi

# Check if .env file exists
if [ ! -f .env ]; then
    echo -e "${YELLOW}.env file not found. Creating from .env.example...${NC}"
    if [ -f .env.example ]; then
        cp .env.example .env
        echo -e "${YELLOW}Please edit the .env file to add your API keys and configuration${NC}"
    else
        echo -e "${RED}.env.example file not found. Please create a .env file with your configuration${NC}"
        exit 1
    fi
fi

# Validate that required environment variables are set
echo "Validating environment variables..."
required_vars=(
    "COHERE_API_KEY"
    "QDRANT_URL"
    "QDRANT_API_KEY"
)

for var in "${required_vars[@]}"; do
    if [ -z "${!var}" ]; then
        echo -e "${RED}Environment variable $var is not set. Please add it to your .env file${NC}"
        exit 1
    fi
done

echo -e "${GREEN}Environment variables validated${NC}"

# Build the Docker images
echo "Building Docker images..."
docker-compose build

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to build Docker images${NC}"
    exit 1
fi

echo -e "${GREEN}Docker images built successfully${NC}"

# Stop existing containers if running
echo "Stopping existing containers..."
docker-compose down --remove-orphans

# Start the services
echo "Starting services..."
docker-compose up -d

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to start services${NC}"
    exit 1
fi

# Wait for services to be healthy
echo "Waiting for services to be healthy..."
sleep 10

# Check if the main application is healthy
max_attempts=30
attempt=1

while [ $attempt -le $max_attempts ]; do
    health_status=$(docker-compose ps app | grep -c "healthy")
    if [ $health_status -eq 1 ]; then
        echo -e "${GREEN}Application is healthy${NC}"
        break
    fi

    if [ $attempt -eq $max_attempts ]; then
        echo -e "${RED}Application failed to become healthy after $max_attempts attempts${NC}"
        docker-compose logs app
        exit 1
    fi

    echo "Waiting for application to be healthy... ($attempt/$max_attempts)"
    sleep 10
    attempt=$((attempt + 1))
done

# Check the status of all services
echo "Checking service status..."
docker-compose ps

echo -e "${GREEN}Deployment completed successfully!${NC}"
echo -e "${GREEN}The RAG Ingestion Pipeline is now running.${NC}"
echo "Application URL: http://localhost:8000"
echo "API Documentation: http://localhost:8000/docs"
echo "Qdrant UI: http://localhost:6333/dashboard"

# Display important information
echo -e "\n${YELLOW}Important:${NC}"
echo "Make sure to:"
echo "1. Verify your API keys in the .env file are correct"
echo "2. Check the application logs if you encounter issues: docker-compose logs app"
echo "3. Monitor the system resources, especially memory usage"
echo "4. Configure SSL/TLS for production environments"

# Display system resources recommendation
echo -e "\n${YELLOW}Recommended system resources:${NC}"
echo "CPU: 4+ cores"
echo "Memory: 8GB+ (Qdrant can be memory intensive)"
echo "Storage: SSD recommended for better performance"

exit 0