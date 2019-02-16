import math

def loadpose(posename):
    f=open(posename)
    line=f.readline()

    time_str=[]
    time=[]
    x=[]
    y=[]
    z=[]
    qx=[]
    qy=[]
    qz=[]
    qw=[]

    while line:
        s=line.split()
        time_str.append(s[0])
        time.append(float(s[0]))
        x.append(float(s[1]))
        y.append(float(s[2]))
        z.append(float(s[3]))
        qx.append(float(s[4]))
        qy.append(float(s[5]))
        qz.append(float(s[6]))
        qw.append(float(s[7]))

        line=f.readline()

    return time_str,time,x,y,z,qx,qy,qz,qw

if __name__ == '__main__':
    camgtout = open('', 'w')
    gpsfile=''
    camfile=''
    gpstime_str, gpstime, gpsx, gpsy, gpsz, gpsqx, gpsqy, gpsqz, gpsqw=loadpose(gpsfile)
    camtime_str, camtime, camx, camy, camz, camqx, camqy, camqz, camqw=loadpose(camfile)
    j = 0
    for i in range(len(camtime)):
        if j>10:
            j=j-10
        while gpstime[j]<=camtime[i]:
            j=j+1
            if j==len(gpstime):
                break
        if j==0:
            if math.fabs(camtime[i]-gpstime[0])>1.5*math.fabs(camtime[i]-camtime[i+1]):
                continue
            outx=gpsx[0]
            outy=gpsy[0]
            outz=gpsz[0]
        elif j==len(gpstime):
            if math.fabs(camtime[i]-gpstime[len(gpstime)-1])>1.5*math.fabs(camtime[i]-camtime[i-1]):
                continue
            outx=gpsx[len(gpstime)-1]
            outy = gpsy[len(gpstime) - 1]
            outz = gpsz[len(gpstime) - 1]
        else:
            inter=(camtime[i]-gpstime[j-1])/(gpstime[j]-gpstime[j-1])
            outx=gpsx[j-1]+inter*(gpsx[j]-gpsx[j-1])
            outy = gpsy[j - 1] + inter * (gpsy[j] - gpsy[j - 1])
            outz = gpsz[j - 1] + inter * (gpsz[j] - gpsz[j - 1])
        wline=camtime_str[i]+' '+str(outx)+' '+str(outy)+' '+str(outz)+' 0 0 0 1\n'
        camgtout.write(wline)
        print(i)

    print('convert done')
