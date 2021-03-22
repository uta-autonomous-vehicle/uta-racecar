import paramiko

class GetZ(object):
    def __init__(self):
        self.url = ''
        self.apikey = ''
        self.uname = ''
        self.passwd = ''
        self.logfile= ''

    def ssh_get_z(self):
        self.url = '10.0.0.104'
        self.apikey = '9BAC3F5894A34FE38DB42B4634685BFA'
        self.uname = 'pi'
        self.passwd = 'MT059Z73'
        self.logfile= '/home/pi/.octoprint/logs/Z.log'
        
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.url, username=self.uname, password=self.passwd)
            ftp = ssh.open_sftp() # Establist SFTP connection
            remote_command = ("tail -n1 " + self.logfile) #Construct terminal command to send
            stdin, stdout, stderr = ssh.exec_command(remote_command) #Send command
            last_line = stdout.read() #Get response of terminal command
            last_line = last_line.rstrip() #Strip newline charachters
            last_line = float(last_line) #Convert Binary response to float
    
            #Close connections and return value
            ftp.close()
            ssh.close()
            
            return last_line
        except Exception as e:
            return None

def main():
    z = GetZ().ssh_get_z()
    print(z) #Print z-height to stdout
    
if __name__ == "__main__":
    main()
