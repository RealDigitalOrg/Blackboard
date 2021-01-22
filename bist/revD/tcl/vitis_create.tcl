###############################################################################
# File: vitis_create.tcl
# Author: Tinghui Wang
#
# Copyright (c) 2019, RealDigital.org
#
# Description:
#   Create Vitis project for BlackBoard BIST Project.
#
# History:
#   11/23/19: Initial release
#  
# License: BSD 3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.
#
###############################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]
set script_file "vitis_create.tcl"

set project_name        {bist}
set project_dir         {bist_workspace}
set xsa_file            {bist_proj/bist.xsa}

proc help {} {
  puts "\nDescription:"
  puts "Create Vitis software project for BlackBoard BIST project."
  puts "Syntax:"
  puts "$script_file -tclargs \[--project_name <name>\]"
  puts "$script_file -tclargs \[--project_dir <path>\]"
  puts "$script_file -tclargs \[--xsa <path>\]"
  puts "$script_file -tclargs \[--help\]"
  puts "Usage:"
  puts "Name                   Description"
  puts "------------------------------------------------------------------"
  puts "\[--project_name <name>\] Create project with the specified name."
  puts "                       Default name is \"bist\"."
  puts "\[--project_dir <path>\]  Determine the Vitis workspace paths wrt \".\"." 
  puts "                       Default project path value is \"./bist_workspace\"."
  puts "\[--xsa <path>\]         Determine the path to hardware specification file." 
  puts "\[--help\]               Print help information for this script"
  puts "------------------------------------------------------------------\n" 
  exit 0
}

if { $::argc > 0 } {
  for {set i 0} {$i < [llength $::argc]} {incr i} {
    set option [string trim [lindex $::argv $i]]
    switch -regexp -- $option {
      "--project_dir"   { incr i; set project_dir [lindex $::argv $i] }
      "--project_name" { incr i; set project_name [lindex $::argv $i] }
      "--xsa" { incr i; set xsa_file [lindex $::argv $i] }
      "--help"         { help }
      default {
        if { [regexp {^-} $option] } {
          puts "ERROR: Unknown option '$option' specified, please type '$script_file -tclargs --help' for usage info.\n"
          return 1
        }
      }
    }
  }
}


puts ""
puts "############################################"
puts "# Create Platform Project"
puts "############################################"
puts ""
puts "create workspace $project_dir"
file mkdir $project_dir
setws $project_dir

platform create -name $project_name -hw $xsa_file -os standalone
platform generate

app create -name "bist" -hw $project_name 
