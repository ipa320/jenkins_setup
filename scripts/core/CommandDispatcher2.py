from Callback import Callback

from subprocess import Popen, PIPE
import fcntl
import os
import time
from collections import deque
import errno

import threading

import logging

class BasicLineWrapper(object):
    def __init__(self, data):
        self.data = data
        self.time = time.time()

    def __repr__(self):
        return '%.6f %s' % (self.time, self.data)

class OutLineWrapper(BasicLineWrapper):
    def __init__(self, data):
        super(OutLineWrapper, self).__init__(data)

    def __repr__(self):
        return 'OUTPUT %s' % super(OutLineWrapper, self).__repr__()


class ErrorLineWrapper(BasicLineWrapper):
    def __init__(self, data):
        super(ErrorLineWrapper, self).__init__(data)

    def __repr__(self):
        return 'ERROR  %s' % super(ErrorLineWrapper, self).__repr__()


class InLineWrapper(BasicLineWrapper):
    def __init__(self, data):
        super(InLineWrapper, self).__init__(data)

    def __repr__(self):
        return 'INPUT  %s' % super(InLineWrapper, self).__repr__()



class _DataThread(threading.Thread, Callback):
    lastGlobalDataTime = time.time()
    lastGlobalDataTimeLock = threading.Lock()

    def __init__(self, **kwargs):
        self.log = logging.getLogger(__name__)
        self.log.debug('init with <%s>', kwargs)

        self.log.debug('call threading.Thread.__init__(self, target=self.run)')
        threading.Thread.__init__(self, target=self.run)
        self.log.debug('call Callback.__init__(self)')
        Callback.__init__(self)

        self.syncThreads=kwargs.get('syncThreads', False)
        self.commandSleepTime = kwargs['commandSleepTime']
        self.timeout = kwargs['timeout']
        self.stdio = kwargs['stdio']
        self.lastDataTime = None


    def updateLastTime(self):
        self.log.debug('syncThreads-active? "%s"', self.syncThreads)
        if self.syncThreads:
            _DataThread.lastGlobalDataTimeLock.acquire()
            _DataThread.lastGlobalDataTime = time.time()
            _DataThread.lastGlobalDataTimeLock.release()
        else:
            self.lastDataTime = time.time()


    def run(self):
        self.log.debug('THREAD STARTED')
        self.updateLastTime()
        while time.time() - self.getLastTime() < self.timeout:
            try:
                data = self.stdio.read()
                self.updateLastTime()
                self.notify(data)
            except IOError as ex:
                if ex.errno is not errno.EWOULDBLOCK:
                    raise ex
            finally:
                time.sleep(self.commandSleepTime)


    def getLastTime(self):
        if self.syncThreads:
            _DataThread.lastGlobalDataTimeLock.acquire()
            retval = float(_DataThread.lastGlobalDataTime)
            _DataThread.lastGlobalDataTimeLock.release()
        else:
            retval = self.lastDataTime
        return retval

class _StdOutHandler(Callback):
    def __init__(self, lineWrapper=None, **kwargs):
        self.log = logging.getLogger(__name__)
        self.log.debug('start logging')
        Callback.__init__(self)
        self.lineWrapper = lineWrapper
        self.stdio = kwargs['stdio']
        self.dataCallbackFnc = kwargs.get('dataCallbackFnc', None)
        #self.stdioData = deque()

        # setting stdio-file in non blocking mode
        fcntl.fcntl(self.stdio.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

        # create data thread and register data callback
        self.stdioThread = _DataThread(**kwargs)
        #self.stdioThread.register(self.stdioData.extend, self)
        self.stdioThread.register(self.storeData, self)

    def storeData(self, data):
        #self.stdioData.extend(self.preparingData(data))
        #self.notify(self.stdioData)
        self.notify(self.preparingData(data))

    def preparingData(self, data):
        return [self.lineWrapper(x) if self.lineWrapper else x for x in data.splitlines()]


class STDIOTYPE(object):
    STDOUT = object()
    STDERR = object()

class CommandDispatcher(Callback):

    def __init__(self, command, stdIOType, command_args=None, cwd=None, **kwargs):
        self.log = logging.getLogger(__name__)
        self.log.debug('foo')

        Callback.__init__(self)

        assert isinstance(stdIOType, list)
        self.timeout = kwargs['timeout']
        self.commandSleepTime = kwargs['commandSleepTime']
        self.stdIOType = stdIOType
        self.stdioData = deque()

        args = [command]
        if command_args:
            if not isinstance(command_args, list):
                command_args = [command_args]
            args.extend(command_args)

        # create configured process pipes
        prockwargs = dict()
        prockwargs['shell'] = False
        prockwargs['stdin'] = PIPE
        prockwargs['stdout'] = PIPE if STDIOTYPE.STDOUT in stdIOType else None
        prockwargs['stderr'] = PIPE if STDIOTYPE.STDERR in stdIOType else None
        prockwargs['cwd'] = cwd

        # create process
        self.proc = Popen(args, **prockwargs)

        # create process handler

        self.stdoutHandle = None
        if STDIOTYPE.STDOUT in stdIOType:
            self.stdoutHandle = _StdOutHandler(stdio=self.proc.stdout, lineWrapper=OutLineWrapper, **kwargs)
            self.stdoutHandle.register(self._newDataCallbackFnc)

        self.stderrHandle = None
        if STDIOTYPE.STDERR in stdIOType:
            self.stderrHandle = _StdOutHandler(stdio=self.proc.stderr, lineWrapper=ErrorLineWrapper, **kwargs)
            self.stderrHandle.register(self._newDataCallbackFnc)

        self.stdioHandle = [self.stdoutHandle, self.stderrHandle]

    def _newDataCallbackFnc(self, lines):
        assert isinstance(lines, list)
        self.log.debug('extending new data')
        self.stdioData.extend(lines)
        self.notify(lines)

    def printData(self, data):
        for d in data:
            print d

    def sendCmd(self, command):
        if isinstance(command, list):
            map(self.sendCmd, command)
            return

        self._newDataCallbackFnc([InLineWrapper(command)])
        cmd = ''.join([command, os.linesep])
        self.proc.stdin.write(cmd)
        time.sleep(self.commandSleepTime)  # waiting for not breaking the pipe


        self._startDataThreads()

    def _startDataThreads(self):
        self.log.debug('try starting threads')
        for stdioHandle in self.stdioHandle:
            if stdioHandle:
                self.log.debug('  check if thead is alive')
                if stdioHandle.stdioThread.isAlive():
                    self.log.warning('    thread still active')
                    stdioHandle.stdioThread.updateLastTime()
                else:
                    self.log.warning('    thread is dead, try starting')
                    try:
                        stdioHandle.stdioThread.start()
                    except RuntimeError as ex:
                        self.log.exception(ex)


    def join(self):
        for stdioHandle in self.stdioHandle:
            stdioHandle.stdioThread.join(self.timeout)


def foo(data):
    foo = data
    #foo = copy.deepcopy(data)
    while len(foo):
        print foo.pop()
    #while len(data):
    #    print data.pop()

if __name__ == '__main__':
    import copy
    FORMAT = '[%(asctime)s %(filename)s(%(lineno)d):%(threadName)s %(name)s.%(funcName)s] %(levelname)s: %(message)s'
    DATEFMT = '%Y%m%d %H:%M:%S'
    logging.basicConfig(format=FORMAT, level=logging.ERROR, datefmt=DATEFMT)

    log = logging.getLogger(__name__)
    log.debug('#'*100)
    log.debug('APPLICATION STARTED')
    log.debug('#'*100)



    stdIOType = [STDIOTYPE.STDOUT, STDIOTYPE.STDERR]
    cd = CommandDispatcher('/bin/zsh', stdIOType, dataCallbackFnc=foo, timeout=4, commandSleepTime=0.2, syncThreads=True)
    cd.register(foo)
    cd.sendCmd('netstat -a | egrep LISTEN; bbb; echo; w; aaaa')
    cd.sendCmd('ps fax')
    cd.join()

    print '='*200
    print 'FIN'