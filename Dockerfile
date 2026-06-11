# Use an image that includes a virtual display server and web VNC client
FROM x11vnc/desktop:latest

# Set the working directory inside the container
WORKDIR /app

# Install Python, pip, tk, and graphic compilation headers
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-tk \
    zlib1g-dev \
    libjpeg-dev \
    libfreetype6-dev \
    && rm -rf /var/lib/apt/lists/*

# Fix python command alias if necessary
RUN ln -sf /usr/bin/python3 /usr/bin/python

# Upgrade pip FIRST so it knows how to find pre-compiled wheels
RUN python -m pip install --upgrade pip

# Copy requirements and install python packages
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy all your drone simulation scripts into the container
COPY . .

# Expose port 8080 for the web-browser VNC interface
EXPOSE 8080