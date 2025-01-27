import paramiko

class SSHClient:
    def __init__(self, host, username, password):
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(host, username=username, password=password)

    def send_command(self, command):
        stdin, stdout, stderr = self.client.exec_command(command)
        output = stdout.read().decode()
        error = stderr.read().decode()
        if error:
            raise Exception(error)
        return output

class SSHServer:
    def __init__(self, port):
        self.server = paramiko.Server()
        self.server.allow_agent = False
        self.server.auth_none = True
        self.server.listen(port)

    def handle_client(self, client):
        channel = client.get_transport().open_channel('session')
        channel.exec_command('bash')
        while True:
            data = channel.recv(1024)
            if not data:
                break
            print(data.decode())
            channel.send(input().encode())
        channel.close()
        client.close()

def main():
    server = SSHServer(2222)
    while True:
        client = server.accept()
        print('Client connected')
        client.handle_client()
        print('Client disconnected')

if __name__ == '__main__':
    main()
