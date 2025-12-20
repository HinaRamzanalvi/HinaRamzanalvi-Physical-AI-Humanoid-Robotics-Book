FROM python:3.9-slim

# Install nodejs
RUN apt-get update && apt-get install -y \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements
COPY requirements.txt .
RUN pip install -r requirements.txt

# Copy the app
COPY . .

# Build the Docusaurus site
RUN npm install
RUN npm run build

# Expose port
EXPOSE 7860

# Run the application
CMD ["python", "app.py"]