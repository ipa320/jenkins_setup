#!/usr/bin/env python
import os, sys, subprocess

VNCPATH = "/opt/TurboVNC/bin"
ERRORSTATUSCODE = 100

def runVNCCmdWithArgs( vncArgs ):
    cmd    = "%s/vncserver %s" % ( VNCPATH, vncArgs )
    args   = cmd.split( ' ' )
    result = subprocess.call( args )
    return result == 0

def startVNConDisplay( i ):
    print 'Starting VNC on Display :%s' % i
    args = ':%s -noauth' % i
    return runVNCCmdWithArgs( args )

def fileExists( filepath ):
    try:
        os.stat( filepath )
        return True
    except OSError,e:
        return False

def xNotLocked( disp ):
    locked1 = fileExists( '/tmp/.X11-unix/X%s' % disp )
    locked2 = fileExists( '/tmp/.X%s-lock' % disp )
    locked  = locked1 or locked2
    return not locked

def getFreeDisplay():
    args = [ 'ps', 'ax' ]
    p    = subprocess.Popen( args, stdout=subprocess.PIPE )
    disp = 1
    stdout, stderr = p.communicate()
    while True:
        if stdout.find( 'Xvnc :%s' % disp ) == -1 and xNotLocked( disp ):
            return disp
        disp += 1

def start():
    disp = getFreeDisplay()
    if not startVNConDisplay( disp ):
        raise Exception( 'Could not start VNC Server' )

    return disp

def stop():
    if not 'DISPLAY' in os.environ:
        print 'DISPLAY not in environment variables found'
        sys.exit( ERRORSTATUSCODE )
    display = os.environ[ 'DISPLAY' ]
    print 'Stopping VNC on Display %s' % display
    args = '-kill %s' % display
    return runVNCCmdWithArgs( args )

def printUsageAndExit():
    print( 'Usage: %s [start|stop :display]. ' % sys.argv[ 0 ] )
    print( 'Returns VNC display number or 100 in case of error' )
    sys.exit( ERRORSTATUSCODE )

def getAction():
    if len( sys.argv ) != 2 or sys.argv[ 1 ] not in ( 'start', 'stop' ):
        printUsageAndExit()
    return sys.argv[ 1 ]

if __name__ == '__main__':
    action = getAction()
    try:
        if action == 'start':
            display = start()
            with file( '/tmp/vncDisplay', 'w' ) as f:
                f.write( ':%s' % display )
        else:
            stop()
            sys.exit( 0 )

    except Exception, e:
        print str( e )
        sys.exit( ERRORSTATUSCODE )
