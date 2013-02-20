#!/usr/bin/env python

# anrmalloc 
#
# Copyright © 2013 Lexmark International
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 2.1 as published by the Free Software Foundation
# (the "LGPL").
#
# You should have received a copy of the LGPL along with this library
# in the file COPYING; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA
#
# This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
# OF ANY KIND, either express or implied.
#
# The Original Code is the anrmalloc and gmalloc API, with anrcore library.
# 
# The Initial Developer of the Original Code is Lexmark International, Inc.
# Author: Ian Watkins
# 
# Commercial licensing is available. See the file COPYING for contact
# information.


import sys
import re
import subprocess


__leak_regex__ = re.compile("Memory Leak:\s+(?P<caller>0x[A-Fa-f0-9]+):size=(?P<size>\d+)")
__track_regex__ = re.compile("pointer=0x[a-fA-F0-9]+\s+size=(?P<size>\d+)\s+location=[(](?P<caller>0x[a-fA-F0-9]+)[)]")
__track2_regex__ = re.compile("pointer=0x[a-fA-F0-9]+\s+size=(?P<size>\d+)\s+location=(?P<caller>0x[a-fA-F0-9]+)\s+count=\d+\s+tid=\d+\s+requested size=\d+")

class MemoryLeak:
    def __init__(self, caller):
        self.caller = caller
        self.size = 0
        self.allocations = 0
    def addLeak(self, size):
        self.size += int(size)
        self.allocations += 1 

def usage():
    print >>sys.stderr, "usage: %s -e [executable] -f [file]" %(sys.argv[0])
    sys.exit(1)


def parse_leaks(leakfile):
    leaks = dict()

    global __leak_regex__, __track_regex__

    for line in file(leakfile).readlines():
        leak_match =  __leak_regex__.match(line)
        track_match = __track_regex__.match(line)
        track2_match = __track2_regex__.match(line)
        group = None           
        if leak_match:
            group = leak_match.groupdict() 
        elif track_match:
            group = track_match.groupdict()
        elif track2_match:
            group = track2_match.groupdict()
        else:
            #print "%s didn't match anything" %line
            pass

        if group: 
            leak = leaks.get(group['caller'], MemoryLeak(group['caller']))
            leak.addLeak(group['size'])
            leaks[group['caller']] = leak
    return leaks 

from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("-f", "--file", action="store", type="string", dest="filename", help="The Memory Leak Report")
    parser.add_option("-e", "--executable", action="store", type="string", dest="program", help="The Program that produced the leaks")

    parser.add_option("-o", "--output", action="store", type="string", dest="output", help="an output file")

    options, args = parser.parse_args()
    
    if not options.filename or not options.program:
        usage()

    leaks = parse_leaks(options.filename)

    caller_list = leaks.keys()
    caller_list.sort(cmp=lambda a,b: leaks[b].size - leaks[a].size)

    callers = "\n".join(caller_list)
    proc = subprocess.Popen(["addr2line", "-C", "-f", "-e",  options.program], stdin=subprocess.PIPE, stdout=subprocess.PIPE)    
    stdout, stderr = proc.communicate("%s" %callers)

    lines = stdout.splitlines()
    funcs = lines[ 0:len(lines):2 ]
    file_and_lines = lines[ 1:len(lines):2 ]

    for caller, func, file_and_line in zip(caller_list, funcs, file_and_lines):
        leak = leaks[caller]
        print "%d Bytes in %d %s from %s at %s (%s)" %(leak.size, leak.allocations, ["allocations", "allocation"][bool(leak.allocations==1)], func, file_and_line, caller)

