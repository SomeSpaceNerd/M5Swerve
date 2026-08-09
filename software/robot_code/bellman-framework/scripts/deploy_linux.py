import paramiko
import json
import os
import time
import sys
from pathlib import Path

# A list of objects in the cwd to send to the remote
UPLOAD_ITEMS = [
    "CMakeLists.txt",
    "robot",
    "bellman-framework",
]

# Makes directories recusively on the remote if they don't exist
def sftp_mkdir_p(sftp, remote_dir):
    try:
        sftp.stat(remote_dir)
        return
    except FileNotFoundError:
        pass

    parent = os.path.dirname(remote_dir)
    if parent and parent != remote_dir:
        sftp_mkdir_p(sftp, parent)

    try:
        sftp.mkdir(remote_dir)
    except OSError:
        pass

# Uploads a directory to the remote recursively 
def upload_dir(sftp, local_dir, remote_dir):
    sftp_mkdir_p(sftp, remote_dir)

    for item in os.listdir(local_dir):
        local_path = os.path.join(local_dir, item)
        remote_path = f"{remote_dir}/{item}"

        if os.path.isdir(local_path):
            upload_dir(sftp, local_path, remote_path)
        else:
            sftp.put(local_path, remote_path)

# Run a command on the remote and stream the output
def run_streaming(prefix, ssh, command):
    channel = ssh.get_transport().open_session()
    channel.get_pty()
    channel.exec_command(command)

    while True:
        if channel.recv_ready():
            print(f"[{prefix}] " + channel.recv(4096).decode("utf-8", errors="replace"), end="", flush=True)

        if channel.recv_stderr_ready():
            print(f"[{prefix}] " + channel.recv_stderr(4096).decode("utf-8", errors="replace"), end="", flush=True)

        if channel.exit_status_ready():
            break

        time.sleep(0.1)

    return channel.recv_exit_status()

def main():
    if len(sys.argv) != 2:
        print("Incorrect number of arguments")
        return

    config_file_path = sys.argv[1]

    try:
        # Attempt to load the config JSON file
        try:
            with open(config_file_path, "r") as file:
                config = json.load(file)
        except FileNotFoundError:
            print("Config JSON file not found")
            return

        # Connect to the device
        print("Connecting to device...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(config["Hostname"], username = config["Username"], password = config["Password"])
        except paramiko.AuthenticationException:
            print("SSH Authentication failed")
            return
        except Exception as e:
            print(f"SSH Connection failed: {e}")
            return

        sftp = ssh.open_sftp() # Open an SFTP connection

        remote_dir = sftp.normalize(".") + "/bellman"
        build_dir = remote_dir + "/build"
        sftp_mkdir_p(sftp, remote_dir)

        local_root = Path(__file__).resolve().parents[2] # Local project root

        # Kill the robot code if it's running
        #print("Killing robot code...")
        #run_streaming(config["Hostname"], ssh, f"pkill -f '{remote_dir}/out/robot_runtime'")

        # Upload the files to the remote
        print("Uploading files to the remote...")
        for item in UPLOAD_ITEMS:
            local_path = os.path.join(local_root, item)

            if not os.path.exists(local_path):
                print(f"Skipping missing item: {item}")
                continue

            remote_path = f"{remote_dir}/{item}"

            if os.path.isdir(local_path):
                upload_dir(sftp, local_path, remote_path)
            else:
                sftp.put(local_path, remote_path)

            print(f"Uploaded: {item}")

        print("Setting up nanopb generator...")
        # For some reason python likes returning weird codes even if it's successful, so we won't check and just barrel along anyway
        run_streaming(config["Hostname"], ssh, "sudo apt update")
        run_streaming(config["Hostname"], ssh, "sudo apt install -y python3-pip build-essential cmake protobuf-compiler libprotobuf-dev dos2unix python3-protobuf python3-grpc-tools python3-grpcio")
        run_streaming(config["Hostname"], ssh, f"""chmod +x {remote_dir}/build/bellman/bellman-protobuf/nanopb/generator/protoc-gen-nanopb """)
        #run_streaming(config["Hostname"], ssh, f"""python3 -m pip install -r {remote_dir}/bellman/bellman-protobuf/nanopb/requirements.txt --break-system-packages""") # --break-system-packages isn't a great idea, but neither is setting up a venv on a potato
        run_streaming(config["Hostname"], ssh, f"dos2unix {remote_dir}/build/bellman/bellman-protobuf/nanopb/generator/protoc-gen-nanopb && dos2unix {remote_dir}/build/bellman/bellman-protobuf/nanopb/generator/nanopb_generator.py") # Because windows is windows, and likes breaking perfectly good software

        # Run CMake
        print("Running CMake...")
        if run_streaming(config["Hostname"], ssh, f"mkdir -p {build_dir} && cmake -S {remote_dir} -B {build_dir} -DCMAKE_BUILD_TYPE=MinSizeRel") != 0 : # Run CMake on the remote
            print("CMake configuration failed")
            return

        # Compile the robot code
        print("Compiling...")
        print("(This may take a while depending on the specs of the remote device)")
        if run_streaming(config["Hostname"], ssh, f"cd {remote_dir} && cmake --build {build_dir} --config MinSizeRel") != 0: # Run CMake --build on the remote
            print("Build failed")
            return

        # Stop robot runtime
        print("Stopping previous robot code...")
        run_streaming(config["Hostname"], ssh, "sudo systemctl stop bellman.service")

        # Copy the robot runtime from out/bin/robot_runtime to /opt/bellman/robot_runtime
        print("Copying executable...")
        if run_streaming(config["Hostname"], ssh, f"mkdir -p /opt/bellman && cp {build_dir}/out/bin/robot_runtime /opt/bellman") != 0:
            print("Copying executable failed")
            return

        # Create and start a systemd service for the robot runtime
        print("Creating systemd service...")
        service_contents = f"""[Unit]
Description=bellman runtime
After=network.target

[Service]
Type=simple
WorkingDirectory={remote_dir}
ExecStart=/opt/bellman/robot_runtime
Restart=always
User={config["Username"]}
StandardOutput=journal
StandardError=journal
SyslogIdentifier=bellman
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=80
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
"""

        # Write the file to a temporary location and copy to proper directory
        with sftp.file("/tmp/bellman.service", "w") as f:
            f.write(service_contents)
        run_streaming(config["Hostname"], ssh, "sudo mv /tmp/bellman.service /etc/systemd/system/bellman.service")

        sftp.close() # Close the SFTP connection

        print("Reloading systemd and enabling service...")
        run_streaming(config["Hostname"], ssh, "sudo systemctl daemon-reload")
        run_streaming(config["Hostname"], ssh, "sudo systemctl enable bellman.service")

        print("Starting robot code...")
        run_streaming(config["Hostname"], ssh, "sudo systemctl restart bellman.service")
        #run_streaming(config["Hostname"], ssh, "cd bellman && ./out/bin/robot_runtime") # Run the robot code

        print("Beginning cout/cerr stream from robot code")
        run_streaming(config["Hostname"], ssh, "sudo journalctl _SYSTEMD_INVOCATION_ID=$(systemctl show bellman -p InvocationID --value) -o cat -f")
    except KeyboardInterrupt:
        print("\nCtrl+C received. Stopping output...")
        print("(Robot code will continue running)")

    # Always cleanup connections
    finally:
        try: 
            sftp.close()
        except:
            pass

        try:
            ssh.close()
        except:
            pass
    
if __name__ == "__main__":
    main()