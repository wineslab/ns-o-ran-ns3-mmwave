import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer

class BashRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        # Get the content length from the headers
        content_length = int(self.headers['Content-Length'])
        # Read the body of the request
        post_data = self.rfile.read(content_length).decode('utf-8')

        # Find and kill the process
        try:
            # Get the process ID(s) for the process with the name or partial name
            result = subprocess.run(f"pgrep -f {post_data}", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
            pids = result.stdout.strip().split('\n')

            if pids and pids[0]:
                # Kill the processes
                for pid in pids:
                    subprocess.run(f"kill {pid}", shell=True)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f"Process(es) with name '{post_data}' terminated.".encode('utf-8'))
            else:
                self.send_response(400)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f"No process found with name '{post_data}'.".encode('utf-8'))
        except Exception as e:
            # Handle any errors
            self.send_response(500)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"Error stopping process: {e}".encode('utf-8'))

    def do_GET(self):
        self.send_response(405)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(b"Use POST to stop processes by partial name.")

def run(server_class=HTTPServer, handler_class=BashRequestHandler, port=38867):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f'Starting httpd on port {port}...')
    httpd.serve_forever()

if __name__ == '__main__':
    run()
