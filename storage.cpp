#include "video_utils.hpp"
#include <QDateTime>
#include <QDir>
#include <QStringList>


QString getStorageFolder(){
    auto mediaRoot = QString("/media/")+QString(qgetenv("USER"));
    QDir directory(mediaRoot);
    QStringList carpetas = directory.entryList(QStringList(),QDir::Dirs|QDir::NoDotAndDotDot);
    if(carpetas.size()>0){
        auto newbase = mediaRoot+"/"+carpetas[0]+"/videosCamaras";
        if(!QDir(newbase).exists()){
            QDir().mkdir(newbase);
        }
        return newbase;
    }

    return ".";
}
