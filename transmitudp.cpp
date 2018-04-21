#include "network_utils.hpp"
#include <vector>
#include <sys/types.h>
#include <string.h>
constexpr int sizePaquetes = 2*2048;

//Los campos a enviar con cada paquete son:
// - offsetDatos
// - totalDataSize
// - ancho
// - alto
// - formato
// - min_value
// - minPosX
// - minPosY
// - depthScale


void transmitirPorUDP(
    int ancho, int alto, int formatoPixel,
    const void* imageData, int rawDataSize,
    int min_value, int minPosX, int minPosY, float depthScale,
    int sockfd, sockaddr_in &serveraddr, int serverlen)
{
    constexpr int headerSize = 4+9*sizeof(int);//startHeader plus 9 data fields
    int nPacks = rawDataSize/sizePaquetes;
    int bytesSobrantes =rawDataSize%sizePaquetes;
    if(bytesSobrantes>0){
            nPacks++;
    }
    int offsetDatos =0;
    int porEnviar =rawDataSize;
    const int totalImageSize=rawDataSize;

    std::vector<unsigned char> bufferTrans;
    bufferTrans.resize(headerSize+sizePaquetes);
    for(int p = 0; p<nPacks; p++){
            unsigned char *datos = bufferTrans.data();
            datos[0] = 0xFC; datos+=1;
            datos[0] = 0xFD; datos+=1;
            datos[0] = 0xFE; datos+=1;
            datos[0] = 0xFF; datos+=1;
            //cout<<"EnviandoOFFSET: "<<offsetDatos<<endl;
            memcpy(datos, &offsetDatos, sizeof(offsetDatos)); datos+=sizeof(offsetDatos);
            memcpy(datos, &totalImageSize, sizeof(totalImageSize)); datos+=sizeof(totalImageSize);
            memcpy(datos, &ancho, sizeof(ancho)); datos+=sizeof(ancho);
            memcpy(datos, &alto, sizeof(alto)); datos+=sizeof(alto);
            memcpy(datos, &formatoPixel, sizeof(formatoPixel)); datos+=sizeof(formatoPixel);
            memcpy(datos, &min_value, sizeof(min_value)); datos+=sizeof(min_value);
            memcpy(datos, &minPosX, sizeof(minPosX)); datos+=sizeof(minPosX);
            memcpy(datos, &minPosY, sizeof(minPosY)); datos+=sizeof(minPosY);
            memcpy(datos, &depthScale, sizeof(depthScale)); datos+=sizeof(depthScale);

            int nDatosAEnviarEnEstePaquete = std::min(sizePaquetes, porEnviar);
            memcpy(
                   datos,
                   ((unsigned char*)imageData) + offsetDatos,
                   nDatosAEnviarEnEstePaquete
                   );datos+=nDatosAEnviarEnEstePaquete;
            const int packSize=nDatosAEnviarEnEstePaquete+headerSize;
            sendto(
                   sockfd,
                   bufferTrans.data(),
                   packSize,
                   0,
                   (sockaddr*)&serveraddr,
                   serverlen
                   );

            porEnviar-=nDatosAEnviarEnEstePaquete;
            offsetDatos+=nDatosAEnviarEnEstePaquete;
    }

}
