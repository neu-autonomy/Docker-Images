#!/bin/bash

# DLIOM with Ouster LiDAR Automation Script
# This script automates the entire setup process for DLIOM with Ouster LiDAR

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

error() {
    echo -e "${RED}[ERROR] $1${NC}"
}

info() {
    echo -e "${BLUE}[INFO] $1${NC}"
}

# Function to detect Ouster LiDAR
detect_ouster() {
    log "Detecting Ouster LiDAR..."
    
    echo "Please check your Ouster device sticker for the serial number."
    echo "Hostname format: os-[serial_number].local"
    read -p "Enter Ouster hostname (e.g., os-1222225000331.local): " hostname
    
    # Validate hostname
    if ping -c 1 -W 2 "$hostname" &> /dev/null; then
        echo "$hostname"
        return 0
    else
        error "Cannot reach $hostname. Please check connection and try again."
        return 1
    fi
}

# Function to get IP from hostname
get_ip_from_hostname() {
    local hostname=$1
    log "Getting IP address for $hostname..."
    
    # Try multiple methods to resolve IP
    local ip=""
    
    # Method 1: Using ping
    ip=$(ping -c 1 "$hostname" 2>/dev/null | grep -oP '(?<=\()[^)]*(?=\))' | head -1)
    
    # Method 2: Using nslookup if ping fails
    if [[ -z "$ip" ]] && command -v nslookup &> /dev/null; then
        ip=$(nslookup "$hostname" 2>/dev/null | grep -A1 "Name:" | grep "Address:" | awk '{print $2}' | head -1)
    fi
    
    # Method 3: Using host command
    if [[ -z "$ip" ]] && command -v host &> /dev/null; then
        ip=$(host "$hostname" 2>/dev/null | grep "has address" | awk '{print $4}' | head -1)
    fi
    
    if [[ -n "$ip" ]]; then
        log "Found IP address: $ip"
        echo "$ip"
        return 0
    else
        error "Could not resolve IP address for $hostname"
        return 1
    fi
}

# Function to check prerequisites
check_prerequisites() {
    log "Checking prerequisites..."
    
    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        error "Docker is not installed. Please install Docker first."
        exit 1
    fi
    
    # Check if Docker is running
    if ! docker info &> /dev/null; then
        error "Docker is not running. Please start Docker service."
        exit 1
    fi
    
    # Check if user is in docker group
    if ! groups | grep -q docker; then
        warn "Current user is not in docker group. You might need to use sudo."
    fi
    
    # Check if idiom-feature-ros2 exists
    if [[ ! -d "dliom-feature-ros2" ]]; then
        error "dliom-feature-ros2 package not found in current directory."
        error "Please download dliom-feature-ros2 to the same folder as Docker images."
        exit 1
    fi
    
    # Check if Dockerfiles exist
    if [[ ! -f "Dockerfile.dliom" ]]; then
        error "Dockerfile.dliom not found in current directory."
        exit 1
    fi
    
    log "All prerequisites met!"
}

# Function to build Docker images
build_docker_images() {
    log "Building Docker images..."
    
    # Build DLIOM image
    if ! docker build -f Dockerfile.dliom -t dliom:latest .; then
        error "Failed to build DLIOM Docker image"
        exit 1
    fi
    
    log "Docker images built successfully!"
}

# Function to setup X11 forwarding
setup_x11() {
    log "Setting up X11 forwarding..."
    
    if [[ -n "$DISPLAY" ]]; then
        xhost +local:docker
        log "X11 forwarding configured"
    else
        warn "DISPLAY variable not set. GUI applications may not work."
    fi
}

# Function to start services using Docker Compose
start_with_compose() {
    log "Starting services with Docker Compose..."
    
    # Enable X11 forwarding
    setup_x11
    
    # Start all services
    if docker-compose up -d; then
        log "All services started successfully!"
        
        # Show running containers
        info "Running containers:"
        docker-compose ps
        
        # Show logs
        info "Starting log output (Ctrl+C to stop viewing logs):"
        docker-compose logs -f
    else
        error "Failed to start services with Docker Compose"
        exit 1
    fi
}

# Function to start services manually
start_manually() {
    local ouster_hostname=$1
    local ouster_ip=$2
    
    log "Starting services manually..."
    
    # Enable X11 forwarding
    setup_x11
    
    # Start Ouster driver container
    log "Starting Ouster driver container..."
    docker run -d \
        --name ouster-driver \
        --network=host \
        --privileged \
        -v /dev:/dev \
        -v /sys:/sys \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY="$DISPLAY" \
        ouster:latest
    
    # Wait for Ouster driver to start
    sleep 10
    
    # Start topic relays
    log "Starting IMU topic relay..."
    docker run -d \
        --name imu-relay \
        --network=host \
        dliom:latest \
        ros2 run topic_tools relay /ouster/imu /ouster/imu_best_effort \
        --qos-reliability best_effort --qos-durability volatile
    
    log "Starting point cloud topic relay..."
    docker run -d \
        --name pointcloud-relay \
        --network=host \
        dliom:latest \
        ros2 run topic_tools relay /ouster/points /ouster/points_best_effort \
        --qos-reliability best_effort --qos-durability volatile
    
    # Wait for relays to start
    sleep 5
    
    # Start DLIOM
    log "Starting DLIOM..."
    docker run -d \
        --name dliom \
        --network=host \
        --privileged \
        -v /dev:/dev \
        -v /sys:/sys \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY="$DISPLAY" \
        dliom:latest \
        ros2 launch direct_lidar_inertial_odometry_and_mapping dliom.launch.py \
        rviz:=true \
        pointcloud_topic:=/ouster/points \
        imu_topic:=/ouster/imu
    
    log "All services started successfully!"
    
    # Show running containers
    info "Running containers:"
    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}

# Function to stop all services
stop_services() {
    log "Stopping all DLIOM services..."
    
    # Try Docker Compose first
    if [[ -f "docker-compose.yml" ]]; then
        docker-compose down
    fi
    
    # Stop individual containers
    containers=("dliom" "pointcloud-relay" "imu-relay" "ouster-driver")
    for container in "${containers[@]}"; do
        if docker ps -q -f name="$container" | grep -q .; then
            docker stop "$container"
            docker rm "$container"
        fi
    done
    
    log "All services stopped"
}

# Function to show logs
show_logs() {
    if [[ -f "docker-compose.yml" ]]; then
        docker-compose logs -f
    else
        info "Available containers:"
        docker ps --format "table {{.Names}}\t{{.Status}}"
        read -p "Enter container name to view logs: " container_name
        docker logs -f "$container_name"
    fi
}

# Function to show help
show_help() {
    echo "DLIOM with Ouster LiDAR Automation Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  start     Start DLIOM with Ouster LiDAR (default)"
    echo "  stop      Stop all services"
    echo "  restart   Restart all services"
    echo "  logs      Show logs from running services"
    echo "  status    Show status of running containers"
    echo "  clean     Clean up all containers and images"
    echo "  help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0          # Start services"
    echo "  $0 start    # Start services"
    echo "  $0 stop     # Stop all services"
    echo "  $0 logs     # View logs"
}

# Function to show status
show_status() {
    info "Docker containers status:"
    docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -E "(ouster|dliom|relay)"
    
    if command -v ros2 &> /dev/null; then
        info "Available ROS2 topics:"
        timeout 5s ros2 topic list | grep -E "(ouster|dliom)" || echo "No ROS2 topics detected"
    fi
}

# Function to clean up
cleanup() {
    log "Cleaning up containers and images..."
    
    # Stop services first
    stop_services
    
    # Remove images
    docker rmi dliom:latest ouster:latest 2>/dev/null || true
    
    # Clean up unused images and containers
    docker system prune -f
    
    log "Cleanup completed"
}

# Main function
main() {
    local command=${1:-start}
    
    case $command in
        start)
            log "Starting DLIOM with Ouster LiDAR setup..."
            
            check_prerequisites
            build_docker_images
            
            # Detect Ouster LiDAR
            ouster_hostname=$(detect_ouster)
            if [[ -z "$ouster_hostname" ]]; then
                error "Failed to detect Ouster LiDAR"
                exit 1
            fi
            
            ouster_ip=$(get_ip_from_hostname "$ouster_hostname")
            if [[ -z "$ouster_ip" ]]; then
                error "Failed to get IP address for Ouster LiDAR"
                exit 1
            fi
            
            info "Ouster LiDAR detected: $ouster_hostname ($ouster_ip)"
            
            # Choose startup method
            if [[ -f "docker-compose.yml" ]]; then
                start_with_compose
            else
                start_manually "$ouster_hostname" "$ouster_ip"
            fi
            ;;
        stop)
            stop_services
            ;;
        restart)
            stop_services
            sleep 2
            main start
            ;;
        logs)
            show_logs
            ;;
        status)
            show_status
            ;;
        clean)
            cleanup
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            error "Unknown command: $command"
            show_help
            exit 1
            ;;
    esac
}

# Handle Ctrl+C gracefully
trap 'echo -e "\n${YELLOW}Interrupted by user${NC}"; stop_services; exit 130' INT

# Run main function
main "$@"
