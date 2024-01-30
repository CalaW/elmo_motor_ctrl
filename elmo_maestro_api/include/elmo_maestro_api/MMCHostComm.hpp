/*
 * CMMCHostComm.hpp
 *
 *  Created on: 24/03/2011
 *      Author: yuvall
 *      Update: Haim H. 03Mar2015 Changes for support Multi type of OS.
 */

#ifndef CHOSTCOMM_HPP_
#define CHOSTCOMM_HPP_

#include "MMCPPGlobal.hpp"
#include "MMCModbusSwapBuffer.hpp"


class DLLMMCPP_API CMMCHostComm {
public:
    CMMCHostComm() {m_uiConnHndl= 0 ;}
    ~CMMCHostComm() {}

    inline void MbusSetConnection(MMC_CONNECT_HNDL uiConnHndl) {m_uiConnHndl = uiConnHndl;}
    void MbusStartServer(MMC_CONNECT_HNDL uiConnHndl, ELMO_UINT16 usID) ;
    void MbusStopServer() ;
    void MbusReadHoldingRegisterTable(int startRef, int refCnt, MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT& stOutParams) ;
    void MbusWriteHoldingRegisterTable(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams) ;
    bool MbusIsRunning(MMC_CONNECT_HNDL uiConnHndl = NULL) ;
    void MbusReadCoilsTable(ELMO_INT32 startRef, ELMO_INT32 refCnt, MMC_MODBUSREADCOILS_OUT& stOutParams) ;
    void MbusWriteCoilsTable(MMC_MODBUSWRITECOILS_IN& stInParams) ;
    void MbusReadInputsTable(ELMO_INT32 startRef, ELMO_INT32 refCnt, MMC_MODBUSREADINPUTS_OUT& stOutParams) ;

    void SetModbusLong(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams, ELMO_INT32 iOffset, ELMO_LINT32 lValue) ;
    void SetModbus(ELMO_INT32 iOffset, ELMO_LINT32 lValue) ;
    void SetModbus(ELMO_INT32 iOffset, ELMO_FLOAT fValue) ;
    void SetModbus(ELMO_INT32 iOffset, ELMO_INT16 sValue) ;

    void SetModbusLongSwapped(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams, ELMO_INT32 iOffset, ELMO_LINT32 lPos) ;
    void SetModbusShort(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams, ELMO_INT32 iOffset, ELMO_INT16 sPos) ;
    void SetModbusFloat(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams, ELMO_INT32 iOffset, ELMO_FLOAT fPos) ;
    void SetModbusFloatSwapped(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN& stInParams, ELMO_INT32 iOffset, ELMO_FLOAT fPos) ;
    void SetModbus(ELMO_INT32 iOffset, ELMO_INT32 iRefCount, CMMCModbusBuffer& stInParams) ;
    void SetModbus(ELMO_INT32 iOffset, ELMO_INT32 iRefCount, CMMCModbusSwapBuffer& stInParams) ;

    void GetModbus(ELMO_INT32 iOffset, ELMO_INT32 iRefCount, CMMCModbusBuffer& stOutParams) ;
    void GetModbus(ELMO_INT32 iOffset, ELMO_INT32 iRefCount, CMMCModbusSwapBuffer& stOutParams) ;


protected:
    //
    void LongToModbusShortArr(ELMO_PINT16 spArr, ELMO_LINT32 lVal)
    {
          *spArr      = (ELMO_INT16) (lVal   & 0xFFFF);
          *(spArr + 1)= (ELMO_INT16)((lVal >> 16) & 0xFFFF);
    }
    void SwapLongToModbusShortArr(ELMO_PINT16 spArr, ELMO_LINT32 lVal)
    {
          *spArr      = (ELMO_INT16)((lVal >> 16) & 0xFFFF);
          *(spArr + 1)= (ELMO_INT16) (lVal & 0xFFFF);
    }

    void FloatToModbusShortArr(ELMO_PINT16 spArr, ELMO_FLOAT fVal)
    {
        union {
            ELMO_FLOAT fVal;
            ELMO_LINT32 lVal;
        } lfloat;
        lfloat.fVal = fVal;

        *spArr      = (ELMO_INT16) (lfloat.lVal & 0xFFFF);
        *(spArr + 1)= (ELMO_INT16)((lfloat.lVal >> 16) & 0xFFFF);
    }
    void SwapFloatToModbusShortArr(ELMO_PINT16 spArr, ELMO_FLOAT fVal)
    {
        union {
            ELMO_FLOAT fVal;
            ELMO_LINT32 lVal;
        } lfloat;
        lfloat.fVal = fVal;
        *spArr      = (ELMO_INT16)((lfloat.lVal >> 16) & 0xFFFF);
        *(spArr + 1)= (ELMO_INT16) (lfloat.lVal & 0xFFFF);
    }

private:
    MMC_CONNECT_HNDL m_uiConnHndl;
};

#endif /* CHOSTCOMM_HPP_ */
