# Security Best Practices for Robotics Applications

This document provides comprehensive security best practices for robotics applications, covering both software and hardware security considerations.

## Overview

Robotics security is a critical aspect of modern robotics development, encompassing protection against unauthorized access, data breaches, physical tampering, and cyber attacks. This guide covers security considerations for all aspects of the Physical AI and Humanoid Robotics curriculum.

## Security Domains in Robotics

### 1. Network Security
- **Communication Protocols**: Securing ROS 2 communications
- **Network Segmentation**: Isolating robot networks
- **Wireless Security**: Protecting wireless connections
- **Firewall Configuration**: Controlling network access

### 2. Software Security
- **Code Security**: Secure coding practices
- **Dependency Management**: Managing third-party libraries
- **Authentication**: User and system authentication
- **Authorization**: Access control mechanisms

### 3. Hardware Security
- **Physical Access Control**: Protecting hardware components
- **Secure Boot**: Ensuring trusted boot processes
- **Hardware Tamper Detection**: Detecting physical tampering
- **Firmware Security**: Protecting firmware integrity

### 4. Data Security
- **Data Encryption**: Protecting sensitive data
- **Privacy Protection**: Safeguarding personal information
- **Data Integrity**: Ensuring data accuracy
- **Audit Trails**: Tracking system activities

## Network Security Best Practices

### 1. ROS 2 Security Configuration

#### A. DDS Security
```yaml
# security_config.yaml - DDS Security Configuration
name: "robot_security"
version: "1.0"

# DomainParticipant security configuration
domain_participant:
  access_control_plugin: "builtin.AccessControl"
  authentication_plugin: "builtin.Authentication"
  cryptography_plugin: "builtin.Cryptographic"

# Access control rules
access_control:
  # Allow only specific participants
  permissions:
    - domain_id: 0
      allow: ["robot_control", "navigation", "perception"]
      deny: ["*"]  # Deny all others by default

# Authentication configuration
authentication:
  # Use certificate-based authentication
  method: "certificate"
  certificate_file: "/etc/ros/security/certificates/robot_cert.pem"
  private_key_file: "/etc/ros/security/keys/robot_key.pem"
  trusted_ca_file: "/etc/ros/security/certificates/ca_cert.pem"

# Cryptography settings
cryptography:
  # Use AES-256 encryption
  cipher_suite: "AES256_GCM"
  key_size: 256
```

#### B. Secure ROS 2 Node Implementation
```python
# secure_node_example.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import ssl
import hashlib
from typing import Optional


class SecureRobotNode(Node):
    """
    Example of a secure ROS 2 node implementation
    """
    def __init__(self):
        super().__init__('secure_robot_node')

        # Initialize security components
        self.security_manager = SecurityManager()
        self.encryption_manager = EncryptionManager()

        # Create secure publisher with authentication
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.secure_publisher = self.create_publisher(
            String,
            'secure_commands',
            qos_profile
        )

        # Create secure subscription with validation
        self.secure_subscriber = self.create_subscription(
            String,
            'secure_status',
            self.secure_status_callback,
            qos_profile
        )

        # Start security monitoring
        self.security_timer = self.create_timer(1.0, self.security_check)

    def secure_status_callback(self, msg):
        """
        Secure callback with message validation
        """
        # Validate message integrity
        if not self.security_manager.validate_message(msg):
            self.get_logger().error("Invalid message received")
            return

        # Decrypt message if encrypted
        decrypted_msg = self.encryption_manager.decrypt(msg.data)
        if decrypted_msg is None:
            self.get_logger().error("Failed to decrypt message")
            return

        # Process the secure message
        self.process_secure_command(decrypted_msg)

    def process_secure_command(self, command: str):
        """
        Process secure commands with authorization
        """
        # Check if command is authorized
        if not self.security_manager.is_authorized_command(command):
            self.get_logger().error(f"Unauthorized command: {command}")
            return

        # Execute command safely
        self.execute_command(command)

    def security_check(self):
        """
        Periodic security checks
        """
        # Check for security violations
        security_status = self.security_manager.get_security_status()
        if not security_status['secure']:
            self.get_logger().warn(f"Security issue detected: {security_status['details']}")


class SecurityManager:
    """
    Security management for ROS 2 nodes
    """
    def __init__(self):
        self.authorized_commands = {
            'move_forward', 'turn_left', 'turn_right', 'stop',
            'take_picture', 'speak', 'listen', 'shutdown'
        }
        self.message_history = []
        self.security_thresholds = {
            'max_commands_per_minute': 60,
            'max_message_size': 1024  # bytes
        }

    def validate_message(self, msg) -> bool:
        """
        Validate message integrity and format
        """
        # Check message size
        if len(str(msg)) > self.security_thresholds['max_message_size']:
            return False

        # Check for command frequency
        current_time = rclpy.time.Time().nanoseconds
        recent_commands = [
            m for m in self.message_history
            if current_time - m['timestamp'] < 60000000000  # 1 minute
        ]

        if len(recent_commands) > self.security_thresholds['max_commands_per_minute']:
            return False

        # Add to history
        self.message_history.append({
            'timestamp': current_time,
            'content': str(msg)
        })

        # Keep only recent history
        self.message_history = self.message_history[-100:]

        return True

    def is_authorized_command(self, command: str) -> bool:
        """
        Check if command is authorized
        """
        # Simple command validation
        return command.split()[0] in self.authorized_commands

    def get_security_status(self) -> dict:
        """
        Get current security status
        """
        return {
            'secure': True,
            'details': 'All systems nominal',
            'last_check': rclpy.time.Time().nanoseconds
        }


class EncryptionManager:
    """
    Encryption management for secure communications
    """
    def __init__(self):
        self.encryption_key = self._generate_encryption_key()
        self.algorithm = 'AES-256'

    def _generate_encryption_key(self) -> bytes:
        """
        Generate secure encryption key
        """
        import os
        return os.urandom(32)  # 256-bit key

    def encrypt(self, data: str) -> str:
        """
        Encrypt data before transmission
        """
        # In practice, use a proper encryption library like cryptography
        import base64
        # This is a simplified example - use proper encryption in production
        encrypted = base64.b64encode(data.encode()).decode()
        return encrypted

    def decrypt(self, encrypted_data: str) -> Optional[str]:
        """
        Decrypt received data
        """
        try:
            import base64
            # This is a simplified example - use proper decryption in production
            decrypted = base64.b64decode(encrypted_data.encode()).decode()
            return decrypted
        except Exception:
            return None
```

### 2. Network Segmentation and Isolation

#### A. Network Configuration
```bash
# network_security.sh - Network security configuration script
#!/bin/bash

# Create isolated network namespace for robot
create_robot_namespace() {
    echo "Creating isolated network namespace for robot..."

    # Create network namespace
    sudo ip netns add robot_ns

    # Create virtual ethernet pair
    sudo ip link add veth0 type veth peer name veth1

    # Move one end to robot namespace
    sudo ip link set veth1 netns robot_ns

    # Configure host side
    sudo ip addr add 192.168.100.1/24 dev veth0
    sudo ip link set veth0 up

    # Configure robot namespace side
    sudo ip netns exec robot_ns ip addr add 192.168.100.2/24 dev veth1
    sudo ip netns exec robot_ns ip link set veth1 up
    sudo ip netns exec robot_ns ip link set lo up

    echo "Network namespace created successfully"
}

# Configure firewall rules
configure_firewall() {
    echo "Configuring firewall rules..."

    # Allow only necessary ROS 2 ports
    sudo ufw allow 11311/tcp  # ROS master
    sudo ufw allow 8883/tcp   # MQTT (if used)
    sudo ufw allow 9090/tcp   # rosbridge (if used)

    # Deny all other incoming connections by default
    sudo ufw default deny incoming
    sudo ufw default allow outgoing

    # Enable firewall
    sudo ufw --force enable

    echo "Firewall configured successfully"
}

# Setup VPN for remote access
setup_vpn_access() {
    echo "Setting up VPN for secure remote access..."

    # Install OpenVPN
    sudo apt update
    sudo apt install -y openvpn easy-rsa

    # Generate certificates (simplified)
    make-cadir ~/openvpn-ca
    cd ~/openvpn-ca
    ./easyrsa init-pki
    ./easyrsa build-ca nopass

    echo "VPN setup initiated - complete certificate generation manually"
}

# Main execution
if [ "$1" = "setup" ]; then
    create_robot_namespace
    configure_firewall
    setup_vpn_access
    echo "Network security setup completed"
elif [ "$1" = "cleanup" ]; then
    sudo ip netns delete robot_ns 2>/dev/null || true
    sudo ip link delete veth0 2>/dev/null || true
    echo "Network security cleanup completed"
else
    echo "Usage: $0 {setup|cleanup}"
    exit 1
fi
```

### 3. Wireless Security for Mobile Robots

#### A. WiFi Security Configuration
```yaml
# wifi_security.yaml - WiFi security configuration
wireless_security:
  # Network configuration
  ssid: "robot_network"
  security_type: "WPA2-Enterprise"  # Use WPA3 if available
  authentication:
    method: "EAP-TLS"  # Certificate-based authentication
    certificate_path: "/etc/robot/security/client_cert.pem"
    private_key_path: "/etc/robot/security/client_key.pem"
    ca_cert_path: "/etc/robot/security/ca_cert.pem"

  # Connection parameters
  connection_timeout: 30  # seconds
  retry_attempts: 3
  reconnect_interval: 5  # seconds

  # Security monitoring
  monitoring:
    scan_interval: 60  # seconds
    suspicious_beacon_threshold: 5
    deauth_attack_detection: true
```

## Software Security Best Practices

### 1. Secure Coding Practices

#### A. Input Validation and Sanitization
```python
# secure_input_handling.py
import re
import html
from typing import Any, Dict, List
import json


class SecureInputHandler:
    """
    Secure input handling with validation and sanitization
    """
    def __init__(self):
        # Define allowed patterns for different input types
        self.patterns = {
            'robot_command': r'^[a-zA-Z_][a-zA-Z0-9_]*$',
            'coordinate': r'^-?\d+\.?\d*,\s*-?\d+\.?\d*(,\s*-?\d+\.?\d*)?$',
            'user_name': r'^[a-zA-Z][a-zA-Z0-9_]{2,19}$',
            'file_path': r'^[a-zA-Z0-9/_\-.]+$'
        }

    def validate_command(self, command: str) -> Dict[str, Any]:
        """
        Validate robot command with security checks
        """
        result = {
            'valid': False,
            'sanitized': '',
            'errors': []
        }

        # Check for command injection patterns
        dangerous_patterns = [
            r'[;&|`$()]',  # Shell metacharacters
            r'\.\./',      # Directory traversal
            r'exec\(|eval\(|system\(',  # Dangerous functions
        ]

        for pattern in dangerous_patterns:
            if re.search(pattern, command, re.IGNORECASE):
                result['errors'].append(f'Dangerous pattern detected: {pattern}')
                return result

        # Validate against allowed pattern
        if not re.match(self.patterns['robot_command'], command.split()[0] if command.split() else ''):
            result['errors'].append('Invalid command format')
            return result

        # Sanitize the command
        result['sanitized'] = self._sanitize_input(command)
        result['valid'] = True

        return result

    def validate_coordinates(self, coord_str: str) -> Dict[str, Any]:
        """
        Validate coordinate input
        """
        result = {
            'valid': False,
            'coordinates': None,
            'errors': []
        }

        if not re.match(self.patterns['coordinate'], coord_str):
            result['errors'].append('Invalid coordinate format')
            return result

        try:
            coords = [float(x.strip()) for x in coord_str.split(',')]
            if len(coords) < 2 or len(coords) > 3:
                result['errors'].append('Invalid number of coordinates')
                return result

            result['coordinates'] = coords
            result['valid'] = True
        except ValueError:
            result['errors'].append('Invalid coordinate values')

        return result

    def _sanitize_input(self, input_str: str) -> str:
        """
        Sanitize input string
        """
        # Remove HTML tags
        sanitized = html.escape(input_str)

        # Remove dangerous characters
        sanitized = re.sub(r'[<>"\']', '', sanitized)

        # Limit length
        if len(sanitized) > 1000:
            sanitized = sanitized[:1000]

        return sanitized

    def validate_json_input(self, json_str: str) -> Dict[str, Any]:
        """
        Validate JSON input safely
        """
        result = {
            'valid': False,
            'data': None,
            'errors': []
        }

        try:
            # Parse JSON with limits to prevent large object attacks
            if len(json_str) > 100000:  # 100KB limit
                result['errors'].append('JSON input too large')
                return result

            parsed_data = json.loads(json_str)

            # Validate structure
            if not isinstance(parsed_data, dict):
                result['errors'].append('JSON must be an object')
                return result

            # Sanitize all string values
            sanitized_data = self._sanitize_dict(parsed_data)

            result['data'] = sanitized_data
            result['valid'] = True
        except json.JSONDecodeError as e:
            result['errors'].append(f'Invalid JSON: {str(e)}')
        except Exception as e:
            result['errors'].append(f'Validation error: {str(e)}')

        return result

    def _sanitize_dict(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recursively sanitize dictionary values
        """
        if isinstance(data, str):
            return self._sanitize_input(data)
        elif isinstance(data, dict):
            return {k: self._sanitize_dict(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [self._sanitize_dict(item) for item in data]
        else:
            return data
```

#### B. Authentication and Authorization
```python
# auth_system.py
import hashlib
import secrets
import jwt
from datetime import datetime, timedelta
from typing import Dict, Optional, List
import bcrypt


class RobotAuthenticationSystem:
    """
    Authentication and authorization system for robotics applications
    """
    def __init__(self):
        self.secret_key = secrets.token_urlsafe(32)
        self.users = {}  # In production, use a database
        self.active_sessions = {}
        self.role_permissions = {
            'admin': ['all_operations'],
            'operator': ['basic_control', 'monitoring'],
            'guest': ['view_only']
        }

    def register_user(self, username: str, password: str, role: str = 'operator') -> bool:
        """
        Register a new user with hashed password
        """
        if username in self.users:
            return False

        # Hash password securely
        password_hash = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())

        self.users[username] = {
            'password_hash': password_hash,
            'role': role,
            'created_at': datetime.now(),
            'last_login': None
        }

        return True

    def authenticate_user(self, username: str, password: str) -> Optional[Dict[str, str]]:
        """
        Authenticate user and return JWT token
        """
        if username not in self.users:
            return None

        user = self.users[username]

        # Verify password
        if bcrypt.checkpw(password.encode('utf-8'), user['password_hash']):
            # Update last login
            user['last_login'] = datetime.now()

            # Generate JWT token
            token_payload = {
                'username': username,
                'role': user['role'],
                'exp': datetime.utcnow() + timedelta(hours=24),
                'iat': datetime.utcnow()
            }

            token = jwt.encode(token_payload, self.secret_key, algorithm='HS256')
            session_id = secrets.token_urlsafe(32)

            # Store session
            self.active_sessions[session_id] = {
                'username': username,
                'token': token,
                'created_at': datetime.now()
            }

            return {
                'session_id': session_id,
                'token': token,
                'role': user['role']
            }

        return None

    def validate_token(self, token: str) -> Optional[Dict[str, str]]:
        """
        Validate JWT token
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=['HS256'])
            return payload
        except jwt.ExpiredSignatureError:
            return None
        except jwt.InvalidTokenError:
            return None

    def authorize_action(self, username: str, action: str) -> bool:
        """
        Check if user is authorized to perform action
        """
        if username not in self.users:
            return False

        user_role = self.users[username]['role']
        user_permissions = self.role_permissions.get(user_role, [])

        # Check if user has permission for action
        return 'all_operations' in user_permissions or action in user_permissions

    def logout_user(self, session_id: str) -> bool:
        """
        Logout user and invalidate session
        """
        if session_id in self.active_sessions:
            del self.active_sessions[session_id]
            return True
        return False


class RBACManager:
    """
    Role-Based Access Control manager
    """
    def __init__(self):
        self.roles = {
            'admin': {
                'permissions': [
                    'robot_control_all',
                    'system_configuration',
                    'user_management',
                    'data_access_all',
                    'security_management'
                ]
            },
            'operator': {
                'permissions': [
                    'robot_basic_control',
                    'navigation',
                    'perception_view',
                    'limited_data_access'
                ]
            },
            'researcher': {
                'permissions': [
                    'experiment_control',
                    'data_analysis',
                    'algorithm_development',
                    'limited_navigation'
                ]
            },
            'guest': {
                'permissions': [
                    'view_system_status',
                    'view_logs',
                    'basic_monitoring'
                ]
            }
        }

    def check_permission(self, role: str, permission: str) -> bool:
        """
        Check if role has specific permission
        """
        role_data = self.roles.get(role, {})
        return permission in role_data.get('permissions', [])

    def get_role_permissions(self, role: str) -> List[str]:
        """
        Get all permissions for a role
        """
        return self.roles.get(role, {}).get('permissions', [])
```

### 2. Dependency and Package Security

#### A. Secure Dependency Management
```python
# dependency_security.py
import hashlib
import requests
from packaging.version import parse, Version
from typing import Dict, List, Optional
import toml


class SecureDependencyManager:
    """
    Secure dependency management for robotics applications
    """
    def __init__(self):
        self.vulnerability_db_url = "https://pyup.io/api/vulnerabilities/"
        self.trusted_sources = [
            "https://pypi.org/simple/",
            "https://test.pypi.org/simple/"
        ]
        self.dependency_cache = {}

    def verify_package_integrity(self, package_name: str, version: str, expected_hash: str) -> bool:
        """
        Verify package integrity using hash comparison
        """
        try:
            # Download package info from PyPI
            response = requests.get(f"https://pypi.org/pypi/{package_name}/{version}/json")
            if response.status_code != 200:
                return False

            package_info = response.json()
            download_url = None

            # Find the correct download URL
            for file_info in package_info['urls']:
                if file_info['filename'].endswith(('.whl', '.tar.gz')):
                    download_url = file_info['url']
                    break

            if not download_url:
                return False

            # Download and hash the file
            file_response = requests.get(download_url)
            actual_hash = hashlib.sha256(file_response.content).hexdigest()

            return actual_hash == expected_hash

        except Exception as e:
            print(f"Error verifying package integrity: {e}")
            return False

    def check_vulnerabilities(self, package_name: str, version: str) -> List[Dict[str, str]]:
        """
        Check for known vulnerabilities in package
        """
        vulnerabilities = []

        try:
            # Check against vulnerability database
            # This is a simplified example - in practice, use a service like PyUp or Snyk
            response = requests.get(f"{self.vulnerability_db_url}{package_name}/")

            if response.status_code == 200:
                vuln_data = response.json()

                for vuln in vuln_data.get('vulnerabilities', []):
                    vuln_version = parse(vuln.get('affected_version', '0.0.0'))
                    if parse(version) >= vuln_version:
                        vulnerabilities.append(vuln)

        except Exception as e:
            print(f"Error checking vulnerabilities: {e}")

        return vulnerabilities

    def generate_secure_requirements(self, requirements_file: str) -> str:
        """
        Generate secure requirements file with version pinning and hashes
        """
        with open(requirements_file, 'r') as f:
            lines = f.readlines()

        secure_lines = []
        for line in lines:
            line = line.strip()
            if line and not line.startswith('#'):
                # Parse package name and version
                if '==' in line:
                    package_name, version = line.split('==')
                else:
                    package_name = line.split('>=')[0].split('<=')[0].split('>')[0].split('<')[0].split('~=')[0]
                    version = None

                # Get latest secure version if not specified
                if not version:
                    version = self.get_latest_secure_version(package_name)

                # Get hash for the specific version
                package_hash = self.get_package_hash(package_name, version)

                if package_hash:
                    secure_lines.append(f"{package_name}=={version} --hash=sha256:{package_hash}")
                else:
                    secure_lines.append(f"{package_name}=={version}")

        secure_requirements = "secure_requirements.txt"
        with open(secure_requirements, 'w') as f:
            f.write('\n'.join(secure_lines))

        return secure_requirements

    def get_latest_secure_version(self, package_name: str) -> str:
        """
        Get the latest version without known vulnerabilities
        """
        try:
            response = requests.get(f"https://pypi.org/pypi/{package_name}/json")
            if response.status_code == 200:
                data = response.json()

                # Get all versions
                versions = list(data['releases'].keys())

                # Sort versions
                sorted_versions = sorted([parse(v) for v in versions], reverse=True)

                # Check each version for vulnerabilities, starting from latest
                for version in sorted_versions:
                    vulns = self.check_vulnerabilities(package_name, str(version))
                    if not vulns:  # No vulnerabilities found
                        return str(version)

                # If all versions have vulnerabilities, return the latest
                return str(sorted_versions[0])

        except Exception as e:
            print(f"Error getting latest secure version: {e}")

        return "latest"

    def get_package_hash(self, package_name: str, version: str) -> Optional[str]:
        """
        Get SHA256 hash for a specific package version
        """
        try:
            response = requests.get(f"https://pypi.org/pypi/{package_name}/{version}/json")
            if response.status_code == 200:
                data = response.json()

                # Find the first .whl or .tar.gz file and get its hash
                for file_info in data['urls']:
                    if file_info['filename'].endswith(('.whl', '.tar.gz')):
                        digests = file_info.get('digests', {})
                        return digests.get('sha256')

        except Exception as e:
            print(f"Error getting package hash: {e}")

        return None
```

## Hardware Security Best Practices

### 1. Physical Security Measures

#### A. Secure Hardware Design
```yaml
# hardware_security.yaml - Hardware security configuration
hardware_security:
  # Physical access control
  physical_access:
    tamper_resistant_enclosure: true
    secure_boot_enabled: true
    hardware_security_module: true
    intrusion_detection: true

  # Secure boot configuration
  secure_boot:
    trusted_execution_environment: true
    measured_boot: true
    verified_boot: true
    rollback_protection: true

  # Hardware security modules
  hsm:
    encryption_engine: true
    key_storage: true
    secure_element: true
    random_number_generator: true

  # Intrusion detection
  intrusion_detection:
    physical_tamper_detection: true
    unauthorized_access_alerts: true
    secure_storage_lock: true
    biometric_access_control: false  # Enable if needed
```

### 2. Firmware Security

#### A. Secure Firmware Updates
```python
# secure_firmware.py
import hashlib
import hmac
import struct
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives import serialization
from typing import Dict, Optional, Tuple
import os


class SecureFirmwareManager:
    """
    Secure firmware management system
    """
    def __init__(self, private_key_path: str, public_key_path: str):
        self.private_key = self._load_private_key(private_key_path)
        self.public_key = self._load_public_key(public_key_path)

    def _load_private_key(self, path: str) -> rsa.RSAPrivateKey:
        """
        Load private key for signing
        """
        with open(path, 'rb') as f:
            private_key = serialization.load_pem_private_key(
                f.read(),
                password=None
            )
        return private_key

    def _load_public_key(self, path: str) -> rsa.RSAPublicKey:
        """
        Load public key for verification
        """
        with open(path, 'rb') as f:
            public_key = serialization.load_pem_public_key(f.read())
        return public_key

    def sign_firmware(self, firmware_path: str) -> Dict[str, bytes]:
        """
        Sign firmware with digital signature
        """
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()

        # Calculate hash of firmware
        firmware_hash = hashlib.sha256(firmware_data).digest()

        # Sign the hash
        signature = self.private_key.sign(
            firmware_hash,
            padding.PKCS1v15(),
            hashes.SHA256()
        )

        return {
            'firmware_hash': firmware_hash,
            'signature': signature,
            'firmware_size': len(firmware_data)
        }

    def verify_firmware_signature(self, firmware_path: str, signature: bytes) -> bool:
        """
        Verify firmware signature
        """
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()

        # Calculate hash of received firmware
        received_hash = hashlib.sha256(firmware_data).digest()

        try:
            # Verify signature
            self.public_key.verify(
                signature,
                received_hash,
                padding.PKCS1v15(),
                hashes.SHA256()
            )
            return True
        except Exception:
            return False

    def create_secure_firmware_package(self, firmware_path: str, output_path: str) -> bool:
        """
        Create a secure firmware package with signature
        """
        try:
            # Sign the firmware
            signature_data = self.sign_firmware(firmware_path)

            # Create package with firmware + signature + metadata
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()

            # Package format: [size][hash][signature][firmware_data]
            size_bytes = struct.pack('<I', len(firmware_data))
            package = (
                size_bytes +
                signature_data['firmware_hash'] +
                signature_data['signature'] +
                firmware_data
            )

            # Write secure package
            with open(output_path, 'wb') as f:
                f.write(package)

            return True

        except Exception as e:
            print(f"Error creating secure firmware package: {e}")
            return False

    def validate_secure_package(self, package_path: str) -> Dict[str, bool]:
        """
        Validate secure firmware package
        """
        try:
            with open(package_path, 'rb') as f:
                package_data = f.read()

            # Parse package: [size:4][hash:32][signature:256][firmware_data]
            size = struct.unpack('<I', package_data[:4])[0]
            firmware_hash = package_data[4:36]
            signature = package_data[36:292]
            firmware_data = package_data[292:]

            # Verify size matches
            if size != len(firmware_data):
                return {'valid': False, 'reason': 'Size mismatch'}

            # Verify signature
            calculated_hash = hashlib.sha256(firmware_data).digest()
            if calculated_hash != firmware_hash:
                return {'valid': False, 'reason': 'Hash mismatch'}

            # Verify signature using public key
            if not self.verify_firmware_signature_from_data(firmware_data, signature):
                return {'valid': False, 'reason': 'Invalid signature'}

            return {'valid': True, 'reason': 'Package is valid'}

        except Exception as e:
            return {'valid': False, 'reason': f'Error validating package: {str(e)}'}

    def verify_firmware_signature_from_data(self, firmware_data: bytes, signature: bytes) -> bool:
        """
        Verify firmware signature from firmware data
        """
        try:
            firmware_hash = hashlib.sha256(firmware_data).digest()
            self.public_key.verify(
                signature,
                firmware_hash,
                padding.PKCS1v15(),
                hashes.SHA256()
            )
            return True
        except Exception:
            return False
```

## Data Security Best Practices

### 1. Data Encryption and Privacy

#### A. Data Protection Implementation
```python
# data_security.py
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
import base64
import os
from typing import Dict, Any, Optional
import json


class DataSecurityManager:
    """
    Data security manager for robotics applications
    """
    def __init__(self):
        self.key_cache = {}

    def generate_key_from_password(self, password: str, salt: bytes = None) -> Tuple[bytes, bytes]:
        """
        Generate encryption key from password
        """
        if salt is None:
            salt = os.urandom(16)

        kdf = PBKDF2HMAC(
            algorithm=hashes.SHA256(),
            length=32,
            salt=salt,
            iterations=100000,
        )
        key = base64.urlsafe_b64encode(kdf.derive(password.encode()))
        return key, salt

    def encrypt_data(self, data: str, password: str) -> Dict[str, str]:
        """
        Encrypt data using password-based key
        """
        key, salt = self.generate_key_from_password(password)
        f = Fernet(key)

        encrypted_data = f.encrypt(data.encode())

        return {
            'encrypted_data': base64.b64encode(encrypted_data).decode(),
            'salt': base64.b64encode(salt).decode()
        }

    def decrypt_data(self, encrypted_package: Dict[str, str], password: str) -> Optional[str]:
        """
        Decrypt data using password
        """
        try:
            salt = base64.b64decode(encrypted_package['salt'])
            encrypted_data = base64.b64decode(encrypted_package['encrypted_data'])

            key, _ = self.generate_key_from_password(password, salt)
            f = Fernet(key)

            decrypted_data = f.decrypt(encrypted_data)
            return decrypted_data.decode()
        except Exception as e:
            print(f"Decryption failed: {e}")
            return None

    def encrypt_robot_data(self, data: Dict[str, Any], encryption_level: str = 'medium') -> Dict[str, Any]:
        """
        Encrypt sensitive robot data based on sensitivity level
        """
        encrypted_data = data.copy()

        # Define sensitive fields that need encryption
        sensitive_fields = {
            'high': ['user_credentials', 'api_keys', 'private_keys'],
            'medium': ['user_data', 'location_data', 'personal_info'],
            'low': ['sensor_data', 'operational_data']
        }

        fields_to_encrypt = sensitive_fields.get(encryption_level, [])

        for field in fields_to_encrypt:
            if field in encrypted_data and isinstance(encrypted_data[field], str):
                encrypted_value = self.encrypt_data(encrypted_data[field], "robot_security_key_2025")
                encrypted_data[field] = {
                    'encrypted': True,
                    'value': encrypted_value['encrypted_data'],
                    'salt': encrypted_value['salt']
                }

        return encrypted_data

    def create_secure_log_entry(self, log_data: Dict[str, Any]) -> str:
        """
        Create a secure log entry with sensitive data protection
        """
        # Remove or encrypt sensitive information
        secure_log = log_data.copy()

        # Sanitize sensitive fields
        sensitive_keys = ['password', 'token', 'key', 'credential', 'secret']
        for key in list(secure_log.keys()):
            if any(sensitive in key.lower() for sensitive in sensitive_keys):
                secure_log[key] = '[REDACTED]'

        # Add timestamp and log level
        secure_log['timestamp'] = str(log_data.get('timestamp', ''))
        secure_log['level'] = log_data.get('level', 'INFO')

        return json.dumps(secure_log, indent=2)

    def implement_data_retention_policy(self, data_path: str, retention_days: int) -> bool:
        """
        Implement data retention policy
        """
        import shutil
        from datetime import datetime, timedelta

        try:
            # Get file modification time
            mod_time = datetime.fromtimestamp(os.path.getmtime(data_path))
            current_time = datetime.now()

            # Check if file is older than retention period
            if current_time - mod_time > timedelta(days=retention_days):
                # Archive or delete old data
                archive_path = f"{data_path}.archive"
                shutil.move(data_path, archive_path)
                print(f"Data archived: {archive_path}")

            return True
        except Exception as e:
            print(f"Error implementing retention policy: {e}")
            return False
```

### 2. Privacy Protection

#### A. Privacy-Enhancing Technologies
```python
# privacy_protection.py
import numpy as np
from typing import List, Dict, Any
import random


class PrivacyProtectionSystem:
    """
    Privacy protection system for robotics applications
    """
    def __init__(self):
        self.differential_privacy_epsilon = 1.0
        self.anonymization_enabled = True

    def add_differential_privacy_noise(self, data: List[float], sensitivity: float = 1.0) -> List[float]:
        """
        Add differential privacy noise to data
        """
        # Calculate noise scale based on epsilon and sensitivity
        noise_scale = sensitivity / self.differential_privacy_epsilon

        # Add Laplace noise
        noisy_data = []
        for value in data:
            noise = np.random.laplace(0, noise_scale)
            noisy_data.append(value + noise)

        return noisy_data

    def anonymize_sensor_data(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Anonymize sensor data to protect privacy
        """
        anonymized_data = sensor_data.copy()

        # Remove or generalize location data
        if 'location' in anonymized_data:
            # Add noise to location coordinates
            if isinstance(anonymized_data['location'], (list, tuple)) and len(anonymized_data['location']) >= 2:
                lat, lon = anonymized_data['location'][0], anonymized_data['location'][1]
                noise_lat = np.random.normal(0, 0.001)  # ~100m radius
                noise_lon = np.random.normal(0, 0.001)
                anonymized_data['location'] = [lat + noise_lat, lon + noise_lon]

        # Generalize timestamp precision
        if 'timestamp' in anonymized_data:
            # Round to nearest minute to prevent precise tracking
            timestamp = anonymized_data['timestamp']
            anonymized_data['timestamp'] = timestamp - (timestamp % 60)

        # Remove unique identifiers
        unique_id_fields = ['user_id', 'device_id', 'session_id']
        for field in unique_id_fields:
            if field in anonymized_data:
                anonymized_data[field] = 'ANONYMIZED'

        return anonymized_data

    def implement_k_anonymity(self, dataset: List[Dict[str, Any]], k: int = 5) -> List[Dict[str, Any]]:
        """
        Implement k-anonymity for dataset
        """
        # Generalize quasi-identifiers to achieve k-anonymity
        generalized_dataset = []

        for record in dataset:
            generalized_record = record.copy()

            # Example: Generalize age to ranges
            if 'age' in generalized_record:
                age = generalized_record['age']
                if age < 20:
                    generalized_record['age_range'] = '0-19'
                elif age < 40:
                    generalized_record['age_range'] = '20-39'
                elif age < 60:
                    generalized_record['age_range'] = '40-59'
                else:
                    generalized_record['age_range'] = '60+'

            # Example: Generalize location to regions
            if 'location' in generalized_record:
                # Convert specific location to broader region
                generalized_record['region'] = self._get_region_from_location(generalized_record['location'])

            generalized_dataset.append(generalized_record)

        return generalized_dataset

    def _get_region_from_location(self, location: Any) -> str:
        """
        Convert specific location to broader region
        """
        # This would typically use a geocoding service
        # For this example, return a generic region
        return "ANONYMIZED_REGION"

    def facial_blurring_simulation(self, image_data: List[List[int]]) -> List[List[int]]:
        """
        Simulate facial blurring for privacy protection
        """
        # This is a simplified simulation
        # In practice, use computer vision libraries like OpenCV
        blurred_image = [row[:] for row in image_data]  # Deep copy

        # Simulate blurring by averaging pixel values in blocks
        block_size = 8
        height, width = len(image_data), len(image_data[0])

        for i in range(0, height, block_size):
            for j in range(0, width, block_size):
                # Calculate average value for the block
                block_sum = 0
                count = 0
                for bi in range(block_size):
                    for bj in range(block_size):
                        if i + bi < height and j + bj < width:
                            block_sum += image_data[i + bi][j + bj]
                            count += 1

                if count > 0:
                    avg_value = block_sum // count
                    # Apply average value to the entire block
                    for bi in range(block_size):
                        for bj in range(block_size):
                            if i + bi < height and j + bj < width:
                                blurred_image[i + bi][j + bj] = avg_value

        return blurred_image
```

## Security Monitoring and Incident Response

### 1. Security Monitoring System

#### A. Security Event Monitoring
```python
# security_monitoring.py
import time
import json
from datetime import datetime
from typing import Dict, List, Any
import logging


class SecurityMonitoringSystem:
    """
    Security monitoring system for robotics applications
    """
    def __init__(self):
        self.security_events = []
        self.alert_thresholds = {
            'failed_login_attempts': 5,
            'unauthorized_access_attempts': 3,
            'network_anomalies': 10,
            'data_access_violations': 2
        }
        self.security_log = []

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('security.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger('RobotSecurity')

    def log_security_event(self, event_type: str, details: Dict[str, Any], severity: str = 'INFO'):
        """
        Log security event
        """
        event = {
            'timestamp': datetime.now().isoformat(),
            'event_type': event_type,
            'details': details,
            'severity': severity
        }

        self.security_events.append(event)
        self.security_log.append(event)

        # Log to file
        self.logger.log(
            getattr(logging, severity.upper(), logging.INFO),
            f"Security Event: {event_type} - {details}"
        )

        # Check for alerts
        self._check_for_alerts(event)

    def _check_for_alerts(self, event: Dict[str, Any]):
        """
        Check if event triggers security alert
        """
        # Count recent events of same type
        recent_events = [
            e for e in self.security_events
            if e['event_type'] == event['event_type']
            and (datetime.now() - datetime.fromisoformat(e['timestamp'])).seconds < 300  # 5 minutes
        ]

        threshold = self.alert_thresholds.get(event['event_type'], 10)
        if len(recent_events) >= threshold:
            self._trigger_security_alert(event['event_type'], len(recent_events))

    def _trigger_security_alert(self, event_type: str, count: int):
        """
        Trigger security alert
        """
        alert = {
            'timestamp': datetime.now().isoformat(),
            'alert_type': 'SECURITY_BREACH_POSSIBLE',
            'event_type': event_type,
            'count': count,
            'action_required': 'INVESTIGATE_IMMEDIATELY'
        }

        self.logger.critical(f"SECURITY ALERT: {event_type} occurred {count} times recently")

        # In a real system, this would trigger notifications, etc.
        print(f"🚨 SECURITY ALERT: {json.dumps(alert, indent=2)}")

    def monitor_network_traffic(self) -> Dict[str, Any]:
        """
        Monitor network traffic for anomalies
        """
        # This would integrate with network monitoring tools
        # For this example, return simulated data
        return {
            'timestamp': datetime.now().isoformat(),
            'traffic_volume': random.randint(100, 1000),
            'connection_count': random.randint(1, 20),
            'anomaly_detected': random.random() < 0.1  # 10% chance of anomaly
        }

    def monitor_system_integrity(self) -> Dict[str, Any]:
        """
        Monitor system integrity
        """
        # Check for unauthorized file changes, running processes, etc.
        # For this example, return simulated data
        return {
            'timestamp': datetime.now().isoformat(),
            'files_changed': 0,
            'unauthorized_processes': 0,
            'system_integrity': 'OK'
        }

    def generate_security_report(self) -> str:
        """
        Generate security report
        """
        report = f"""
# Security Report - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## Summary
- Total Security Events: {len(self.security_events)}
- Security Alerts Triggered: {len([e for e in self.security_events if e['severity'] == 'CRITICAL'])}
- Monitoring Period: Last 24 hours

## Recent Events
"""

        recent_events = [
            e for e in self.security_events
            if (datetime.now() - datetime.fromisoformat(e['timestamp'])).days < 1
        ][:10]  # Last 10 events

        for event in recent_events:
            report += f"- {event['timestamp']}: {event['event_type']} ({event['severity']})\n"

        report += f"""
## Security Status
- Network Security: {'MONITORED' if any('network' in e['event_type'] for e in recent_events) else 'OK'}
- Access Control: {'ACTIVE' if any('login' in e['event_type'] for e in recent_events) else 'OK'}
- Data Protection: {'ACTIVE' if any('data' in e['event_type'] for e in recent_events) else 'OK'}

## Recommendations
"""

        if any(e['severity'] == 'CRITICAL' for e in recent_events):
            report += "- Immediate security review required\n"
        if any('anomaly' in e['event_type'] for e in recent_events):
            report += "- Investigate network anomalies\n"

        report += "- Continue regular security monitoring\n"

        return report
```

This comprehensive security best practices guide covers all major aspects of security for robotics applications, including network security, software security, hardware security, data security, and monitoring. It provides practical examples and implementation guidance for securing robotics systems throughout the Physical AI and Humanoid Robotics curriculum.