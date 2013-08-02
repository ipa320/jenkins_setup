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

def start():
    success = False
    for i in xrange( 1, 20 ):
        if startVNConDisplay( i ): 
            success = True
            break
        print 'Trying next display'

    if not success:
        raise Exception( 'Could not start VNC Server' )
    return i

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
            sys.exit( display )
        else:
            stop()
            sys.exit( 0 )

    except Exception, e:
        print str( e )
        sys.exit( ERRORSTATUSCODE )
