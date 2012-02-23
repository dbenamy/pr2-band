#!/usr/bin/env python

from copy import copy
import logging
from logging import debug, info, warning
import os
import subprocess
import sys
from time import sleep

def usage():
    print "ros_dev_builder.py [--debug] <stack or package dir>"
    print ""
    print "Watches a ROS stack or package for changes to files. If any source"
    print "files are changed, it rebuilds the executables. If any executables"
    print "are changed (including because they were rebuilt), it kills any"
    print "running instances of the executable."
    print ""
    print "You should set up your ROS launch files to respawn nodes that die"
    print "so the new executable gets run when the old one is killed."

def main():
    print "" # Makes it easier to read output
    if len(sys.argv) == 1 or '--help' in sys.argv:
        usage()
        sys.exit(1)
    warning("Python programs won't be killed when they're changed (yet).\n")
    log_level = logging.DEBUG if '--debug' in sys.argv else logging.INFO
    logging.basicConfig(level=log_level)
    watch_dir = sys.argv[1]
    while watch_dir.endswith('/'):
        watch_dir = watch_dir[:-1]
    info("Watching %s." % os.path.abspath(watch_dir))
    file_db = {}
    
    while True:
        changed_sources = []
        changed_executables = []
        makes_run = set()
        
        changed_sources, changed_executables = scan_files(watch_dir, file_db)
        if len(changed_sources) > 0:
            info("These source files changed: %s" % changed_sources)
        debug("These executables changed: %s" % changed_executables)
        make_files = [make_which_builds(f, watch_dir) for f in changed_sources]
        for make in make_files:
            if make not in makes_run:
                makes_run.add(make)
                make_dir = os.path.dirname(make)
                subprocess.call("set -x; cd %s && make" % make_dir, shell=True)
        
        if makes_run:
            changed_sources2, changed_executables2 = scan_files(watch_dir,
                                                                file_db)
            # Don't care about new changed sources
            changed_executables.extend(changed_executables2)
        if len(changed_executables) > 0:
            info("These executables changed: %s" % changed_executables)
        for executable in changed_executables:
            subprocess.call("set -x; fuser -k %s" % executable, shell=True)
            # TODO this won't kill python scripts. figure out how.
        
        if not changed_sources and not changed_executables:
            sleep(1)

def scan_files(top_dir, file_db):
    changed_sources = []
    changed_executables = []
    
    for root, dirs, files in os.walk(top_dir):
      for file_name in files:
        path = os.path.join(root, file_name)
        debug("Looking at %s." % path)
        mtime = os.path.getmtime(path)
        if path not in file_db or file_db[path] != mtime:
            debug("%s was added or changed." % path)
            file_db[path] = mtime
            if path.endswith('.py') or os.path.split(root)[1] == 'bin':
                debug("Adding %s to the changed executables list." % path)
                changed_executables.append(path)
            else:
                debug("Adding %s to the changed sources list." % path)
                changed_sources.append(path)
    return changed_sources, changed_executables

def make_which_builds(path, watch_dir):
    debug("Looking for Makefile which builds %s." % path)
    orig_path = copy(path)
    assert(path.startswith(watch_dir))
    while path.startswith(watch_dir):
        path = os.path.split(path)[0]
        to_check = '%s/Makefile' % path
        debug("Looking for %s." % to_check)
        if os.path.exists(to_check):
            return to_check
    raise Exception("Can't find Makefile to build %s." % orig_path)

if __name__ == '__main__':
    main()

"""
You can set up a test package structure by running the following.

mkdir -p test_pkg/src
mkdir -p test_pkg/bin
echo "#include <iostream>

using namespace std;

int main() {
    int x;
    cin >> x;
    return 0;
}
" > test_pkg/src/main.cpp
echo "all:
	g++ src/main.cpp -o bin/main
" > test_pkg/Makefile
"""
