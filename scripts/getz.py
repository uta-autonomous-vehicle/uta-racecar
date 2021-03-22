#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 14 15:31:45 2021

@author: Travis Major


Description: The z-height output of the M114 g-code command is logged to a
    a file called Z.log in ~/.octoprint/logs (or as specified)
    automatically using an event-fire subscription in Octoprints config.yaml.
    This program creates an ssh session to the printer, opens the file, processes the
    contents, and returns the last Z: (height) value logged.

Prerequisites:  1) Paramiko (python SSH library)
                2) Create a log file named Z.log at 
                ~/.octoprint/logs or modify path below for other
                name or path.

"""

import paramiko

class GetZ(object):
    def __init__(self):
        self.url = ''
        self.apikey = ''
        self.uname = ''
        self.passwd = ''
        self.logfile= ''

    def ssh_get_z():
        self.url = '10.250.250.1'
        self.apikey = '9BAC3F5894A34FE38DB42B4634685BFA'
        self.uname = 'pi'
        self.passwd = 'MT059Z73'
        self.logfile= '/home/pi/.octoprint/logs/Z.log'
        
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.url, username=self.uname, password=self.passwd)
            ftp = ssh.open_sftp() # Establist SFTP connection
            remote_command = ("tail -n1 " + logfile) #Construct terminal command to send
            stdin, stdout, stderr = ssh.exec_command(remote_command) #Send command
            last_line = stdout.read() #Get response of terminal command
            last_line = last_line.rstrip() #Strip newline charachters
            last_line = float(last_line) #Convert Binary response to float
    
            #Close connections and return value
            ftp.close()
            ssh.close()
            return last_line
        except:
            return None

def main():
    z = ssh_get_z()
    print(z) #Print z-height to stdout
    
if __name__ == "__main__":
    main()
