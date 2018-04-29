#include "video_utils.hpp"
#include <QProcess>
#include <QStringList>


int detectUVCCamera(){
    QProcess queryDevs;
    queryDevs.start("v4l2-ctl", QStringList("--list-devices"));
    queryDevs.waitForFinished(-1);
    QString resultado = QString(queryDevs.readAllStandardOutput());
    auto lineasRes = resultado.split("\n");
    int ret = -1;
    bool nextIsIndex = false;
    for(auto& linea : lineasRes){
        if(linea.startsWith("UVC Camera")){
            nextIsIndex = true;
            continue;
        }
        if(nextIsIndex){
            auto lineaDev = linea.trimmed();
            lineaDev = lineaDev.mid(10, 1);
            ret = lineaDev.toInt();
            break;
        }
    }
    return ret;
}
