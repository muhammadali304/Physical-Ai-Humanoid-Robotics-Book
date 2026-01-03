from fastapi import FastAPI
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware

# Load environment variables
load_dotenv()

# Initialize logging configuration first
from src.config.logging_config import setup_logging, get_structured_logger
from src.config.settings import settings

# Set up centralized logging configuration
setup_logging()

# Initialize the application logger
app_logger = get_structured_logger("main")
app_logger.info("Initializing RAG Chatbot API", extra={
    "context": {
        "version": "0.1.0",
        "event_type": "app_initialization"
    }
})

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for conversational RAG chatbot using OpenAI Agents SDK with Gemini model",
    version="0.1.0",
    docs_url="/docs",  # Swagger UI
    redoc_url="/redoc"  # ReDoc UI
)

app_logger.info("FastAPI application initialized", extra={
    "context": {
        "event_type": "app_initialized"
    }
})

# Add CORS middleware (should be one of the first middlewares)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers that frontend might need to access
    expose_headers=["Content-Disposition", "X-Total-Count"]
)

# Add API versioning (should be one of the first middlewares)
from src.api.versioning import setup_versioning
setup_versioning(app)

# Add authentication middleware (for protected routes)
from src.api.middleware.auth import add_authentication_middleware
# Protect all /api/v1/ routes except /api/v1/query for ChatKit integration
add_authentication_middleware(app, protected_routes=["/api/v1/"], excluded_routes=["/api/v1/query"], require_auth_by_default=False)

# Add rate limiting middleware (to catch requests early)
from src.api.middleware.rate_limit import add_rate_limiting_middleware
add_rate_limiting_middleware(app)

# Add logging middleware (for request tracking)
from src.api.middleware.logging import add_request_response_logging, add_performance_logging
add_request_response_logging(app, detailed=True, log_request_body=False, log_response_body=False)
add_performance_logging(app)

# Add error handling middleware (to catch any errors)
from src.api.middleware.error_handler import add_error_handling_middleware
add_error_handling_middleware(app)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API - Conversational RAG with Gemini"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Include API routes
from src.api.routes.query import include_router as include_query_router

# Register the routers
include_query_router(app)

# Log application startup
app_logger.info("RAG Chatbot API initialized successfully", extra={
    "context": {
        "event_type": "app_startup_complete",
        "routes_count": len(app.routes),
        "settings_log_level": settings.log_level,
        "log_json_format": settings.log_json_format,
        "log_file": settings.log_file
    }
})

if __name__ == "__main__":
    import uvicorn
    app_logger.info("Starting RAG Chatbot server", extra={
        "context": {
            "event_type": "server_start",
            "host": "0.0.0.0",
            "port": 8000,
            "log_level": settings.log_level
        }
    })
    uvicorn.run(app, host="0.0.0.0", port=8000)