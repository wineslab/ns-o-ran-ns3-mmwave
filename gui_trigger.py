import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer

# Function to run startup commands
def run_startup_commands():
    commands = [
        'python3 sim_data_pusher.py',
        'python3 stop_ns3.py'
    ]

    for command in commands:
        print(f"Starting command: {command}")
        log_file = command.split()[1].replace('.py', '.log')  # Create log file based on the script name
        with open(log_file, 'w') as log:
            subprocess.Popen(command, shell=True, stdout=log, stderr=log, executable='/bin/bash')


# Define the request handler
class BashRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode('utf-8').strip()

        print(f"Received POST data: {post_data}")

        try:
            # Check if the process 'scenario-zero-w' is already running
            check_command = "ps -a | grep -F 'scenario-zero-w' | grep -v grep"
            result = subprocess.run(check_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            grep_output = result.stdout.strip()

            if grep_output:
                # Process is already running, return a 500 response
                print(f"Process 'scenario-zero-w' is already running:\n{grep_output}")
                self.send_response(500)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f"Error: Process 'scenario-zero-w' is already running:\n{grep_output}".encode('utf-8'))
                return

            # Log file for the process
            log_file = 'ns3_run.log'  # Fixed log file name for the process
            with open(log_file, 'w') as log:
                print(f"Starting process: {post_data}, logging to: {log_file}")
                subprocess.Popen(post_data, shell=True, stdout=log, stderr=log, executable='/bin/bash')

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"Process started.")
        except Exception as e:
            print(f"Error: {e}")
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"Error starting process: {e}".encode('utf-8'))

    def do_GET(self):
        print("Received GET request.")
        self.send_response(405)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(b"Use POST to start bash commands.")


def run(server_class=HTTPServer, handler_class=BashRequestHandler, port=38866):
    run_startup_commands()

    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting httpd on port {port}...')
    httpd.serve_forever()


if __name__ == '__main__':
    run()
