import os
from starlette.staticfiles import StaticFiles
import uvicorn
from fastapi import FastAPI
import subprocess

# Build the Docusaurus site if not already built
def build_site():
    if not os.path.exists("build"):
        print("Building Docusaurus site...")
        # Ensure npm is available and install dependencies
        subprocess.run(["npm", "install"], check=True)
        result = subprocess.run(["npm", "run", "build"], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Build failed: {result.stderr}")
            raise Exception(f"Build failed: {result.stderr}")
        print("Build completed successfully")

# Check if we need to build the site
if not os.path.exists("build"):
    build_site()

# Create FastAPI app to serve static files
app = FastAPI()

# Mount the built Docusaurus site
app.mount("/", StaticFiles(directory="build", html=True), name="static")

@app.get("/")
async def read_root():
    return {"message": "Physical AI & Humanoid Robotics Course Docusaurus site"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))