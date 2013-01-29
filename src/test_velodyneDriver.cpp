// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Tully Foote
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* *********************************************************************************
 * 
 * Test program for using velodyne
 * 
 */
#include <getopt.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <netdb.h>
#include <stdio.h>
#include <string>

#include "velodyneDataDriver.hpp"

using namespace std;

//#define VELODYNE_BENCHMARKING  //Uncomment this to turn on benchmarkign

// FUNCTIONS REQUIRED FOR COMMAND-LINE OPTION HANDLING

// prints usage information for this program to STREAM (typically stdout or stderr), and exits the program
// with exit code = exit_code (0 -> OK exit, -1 -> abortive exit) , DOES NOT RETURN!
void print_usage(int exit_code)
{
  cout << "\nWelcome to test-velodyneDriver" << endl;
  cout << "\nThis is a very simple test program to dump" << endl;
  cout << "\nvelodyne data to verify that it is being recieved." << endl;
  cout << "----------------------------------- COMMAND-LINE OPTIONS ---------------------------------" << endl;
  cout << "  --help,                  -h                 Display this message.                       " << endl;
  cout << "\n" << endl;
  exit(exit_code);
}

int main(int argc, char **argv) // need these args for getopt(-long)
{
  
  // -- COMMAND LINE OPTION HANDLING
  
  // command-line option default value assignments

  // temporary character
  char ch;  
  // a string listing valid short options letters, a : after the character indicates that this option
  // requires an argument, a :: after a character indicates that the argument is optional
  const char* const short_options = "h";
  // an array describing valid long options
  static struct option long_options[] = 
    {
      // first: long option (--option) string
      // second: 0 = no_argument, 1 = required_argument, 2 = optional_argument
      // third: if pointer, set variable to value of fourth argument
      //        if NULL, getopt_long returns fourth argument
      {"help",        0, NULL,              'h'},
      {NULL,          0, NULL,               0}
    };
  
  // Loop through and process all of the command-line input options.
  while( (ch = getopt_long(argc, argv, short_options, long_options, NULL)) != -1 ) {
    switch(ch) {
    case 'h':
      print_usage(0);
      // print_usage(..) doesn't return - program exits
      break; // this is unnecessary except to suppress g++ errors
    case '?': // user specified an invalid/unknown option
      // print the help page and make a graceful abortive exit
      cerr << "ERROR: you specified an unrecognised option, valid options are: \n" << endl;
      print_usage(-1);
    case -1: // user didn't specify any options - you would put default startup options here.
      break;
    default:
      // shouldn't ever get here
      cerr << "ERROR: test_velodyneDriver - parse input options fell through!" << endl;
      exit(-1);
    }
  }
  

  velodyne_lidar::VelodyneDataDriver m_velodyneDriver;
  
  m_velodyneDriver.openUDP("", velodyne_lidar::VELODYNE_DATA_UDP_PORT);
  
  cerr << "test_velodyneDriver starting active loop..." << endl;
  velodyne_lidar::velodyne_data_packet_t temp_packet;
  while (true)
    {
      m_velodyneDriver.readPacket((uint8_t*)&temp_packet, velodyne_lidar::VELODYNE_DATA_MSG_BUFFER_SIZE, 100000, 100000);
      m_velodyneDriver.print_packet(temp_packet);
    }
  return 0; 
}
