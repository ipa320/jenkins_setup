#!/usr/bin/env python

from jenkins_setup import common, cob_pipe


def main():
    pass


if __name__ == "__main__":
    # global try
    try:
        main()
        print "Graphic test script finished cleanly!"

    # global catch
    except (common.BuildException, cob_pipe.CobPipeException) as ex:
        print "Graphic test script failed!"
        print ex.msg
        raise ex

    except Exception as ex:
        print "Graphic test script failed! Check out the console output above for details."
        raise ex
