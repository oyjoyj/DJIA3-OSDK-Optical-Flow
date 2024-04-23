/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*TODO:flight_control_sample will by replace by flight_sample in the future*/
#include "flight_control_sample.hpp"
#include "telemetry_sample.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

using namespace std;
using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


int main(int argc, char** argv) {
  // // Initialize variables
  int functionTimeout = 1;
  float gllocate = 0;
  float glset = 0;
  int glsetflag = 0;
  float glxget = 0,glyget = 0;
  float glvx = 0, glvy = 0, glvz = 0;
  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  

  int fd1, fd2;
    fd_set rfds;
    struct timeval tv;
    int retval;
    char buf[3000];
    int len;
    struct termios options;
    fd1 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd1 == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
        return (-1);
    }
    fcntl(fd1, F_SETFL, 0);
    tcgetattr(fd1, &options);
    cfsetispeed(&options, B500000);
    cfsetospeed(&options, B500000);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd1, TCSANOW, &options);
    fd2 = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd2 == -1)
    {
        perror("open_port: Unable to open /dev/ttyUSB1 - ");
        return (-1);
    }
    fcntl(fd2, F_SETFL, 0);
    tcgetattr(fd2, &options);
    cfsetispeed(&options, B230400);
    cfsetospeed(&options, B230400);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd2, TCSANOW, &options);
    //配置串口

  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }//验证连接飞控
//x y z 轴pid赋值
  float x_kp = 2;//3
  float x_ki = 0;
  float x_kd = 1.83;

  float y_kp = 2;//3
  float y_ki = 0;
  float y_kd = 1.85;

  float z_kp = 5;//3
  float z_ki = 0.05;//0
  float z_kd = 0.53;//0.53

  pid_p *pid = new pid_p [3];
  pid_init(&pid[0], x_kp, x_ki, x_kd);//0是x轴
  pid_init(&pid[1], y_kp, y_ki, y_kd);//1是y轴
  pid_init(&pid[2], z_kp, z_ki, z_kd);//2是z轴
  //初始化pid类
  
    while (1)
    { 
        FD_ZERO(&rfds);
        FD_SET(fd1, &rfds);
        FD_SET(fd2, &rfds);
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        retval = select(fd2 + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
        {
            perror("select()");
        }
        else if (retval)
        {
            if (FD_ISSET(fd1, &rfds))
            {
                len = read(fd1, buf, sizeof(buf));
                if (len > 0)
                {
                    buf[len] = '\0';
                    for(int i = 0;i < len;i++)
                    {
                      if (buf[i] == 0x34 && buf[i+1] == 0x07 && buf[i+2] == 0x00 && buf[i+3] == 0x00 && buf[i+4] == 0x00 && buf[i+6] == 0x00 && buf[i+7] == 0x00 && buf[i+8] == 0x00)
                      {
                        if(buf[i+5] != 0xffffffff){
                          gllocate = buf[i+5];
                        }
                      }//筛选高度数据
                      if (buf[i] == 0x51 && buf[i+1] == 0x05 && buf[i+2] == 0x00 && buf[i+3] == 0x01 && buf[i+6] >= 0xffffff7d)
                      {
                        glxget = buf[i+4];
                        glyget = buf[i+5];
                      }//筛选vx,vy数据
                    }
                }
            }
            glvx = pid_calc(&pid[0], 0, glxget);
            glvy = pid_calc(&pid[1], 0, glyget);
            gllocate = filter(glset, gllocate);
            glvz = pid_calc(&pid[2], 0, glset - gllocate);
            std::cout<<"\n\n\nglset"<<glset<<std::endl;
            std::cout<<"\n\n\ngllocate"<<gllocate<<endl;
        }
      subscribeToData(vehicle,&glset,&glsetflag,&gllocate,&glvx,&glvy,&glvz);
    }
}

//ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(1);//起飞指令