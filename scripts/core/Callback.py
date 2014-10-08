import logging

class Callback(object):
    def __init__(self):
        self.log = logging.getLogger(__name__)
        self.log.debug('logging started')
        self.fncList = dict()

    def register(self, callbackFnc, id=None):
        self.log.debug('callbackFnc "%s" id "%s"', callbackFnc, id)
        if id is None:
            id = object()
            self.log.debug('creating new id "%s" asd "%s"', id, id)
        self.fncList[id] = callbackFnc
        return id

    def unregister(self, id):
        self.log.debug('check id id-key "%s" is in fncList "%s"', id, self.fncList.keys())
        if id in self.fncList.keys():
            self.fncList.pop(id, None)

    def notify(self, msg):
        self.log.debug('trigger callback-functions "%s"', self.fncList.values())
        for fnc in self.fncList.values():
            self.log.debug('call function "%s"', fnc)
            fnc(msg)
        return id