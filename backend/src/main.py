from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from starlette.responses import JSONResponse
import os
import logging
from dotenv import load_dotenv
from src.api import chat_endpoints, health_endpoints

# Load env
load_dotenv()

# Logging (VERY IMPORTANT)
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize database and vector store with mock data
try:
    from src.config.database import engine, Base
    from src.config.vector_db import initialize_vector_db
    from src.scripts.mock_textbook_data import add_mock_textbook_data

    # Create all tables
    Base.metadata.create_all(bind=engine)
    logger.info("Database tables created/verified")

    # Initialize vector database
    initialize_vector_db()
    logger.info("Vector database initialized")

    # Add mock textbook data if not already present
    add_mock_textbook_data()
    logger.info("Mock textbook data added/verified")

except Exception as e:
    logger.error(f"Error initializing database: {str(e)}")

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json"
)

# -------------------- CORS --------------------
allowed_origins = os.getenv(
    "ALLOWED_ORIGINS",
    "http://localhost:3000,http://127.0.0.1:3000,http://localhost:3001,http://127.0.0.1:3001,http://localhost:3002,http://127.0.0.1:3002,http://localhost:3003,http://127.0.0.1:3003"
).split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------- Trusted Hosts --------------------
allowed_hosts = os.getenv(
    "ALLOWED_HOSTS",
    "localhost,127.0.0.1,0.0.0.0"
).split(",")

if "testserver" not in allowed_hosts:
    allowed_hosts.append("testserver")

app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=allowed_hosts
)

# -------------------- Routes --------------------
app.include_router(
    health_endpoints.router,
    prefix="/api/v1",
    tags=["health"]
)

app.include_router(
    chat_endpoints.router,
    prefix="/api/v1",
    tags=["chat"]
)


# -------------------- Exception Handlers --------------------

@app.exception_handler(StarletteHTTPException)
async def http_exception_handler(request, exc):
    logger.error(f"HTTP error: {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": {
                "message": exc.detail,
                "status_code": exc.status_code
            }
        }
    )

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    logger.error(f"Validation error: {exc.errors()}")
    return JSONResponse(
        status_code=422,
        content={
            "error": {
                "message": "Invalid request data",
                "details": exc.errors()
            }
        }
    )

@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    logger.exception("Unhandled server error")
    return JSONResponse(
        status_code=500,
        content={
            "error": {
                "message": "Internal server error",
                "details": str(exc)
            }
        }
    )

# -------------------- Root --------------------
@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running"}

# -------------------- Run --------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
