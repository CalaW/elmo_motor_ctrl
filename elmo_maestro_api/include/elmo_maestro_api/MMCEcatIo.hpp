/*
 * CMMCECATIO.h
 *
 *  Created on: 27/06/2011
 *      Author: yuvall
 *      Update: Haim H. 16Sep2019 Changes for support Multi type of OS.
 */

#ifndef CMMCECATIO_H_
#define CMMCECATIO_H_

#include "MMCGroupAxis.hpp"
#include "MMCNode.hpp"

/*
 *
 */
class DLLMMCPP_API CMMCECATIO : public CMMCNode {
public:
    CMMCECATIO();
    virtual ~CMMCECATIO();

    void        ECATIOEnableDIChangedEvent  (                                       )   ;
    void        ECATIODisableDIChangedEvent (                                       )   ;
    MMCPPULL_T  ECATIOReadDigitalInput      (                                       )   ;
    void        ECATIOWriteDigitalOutput    (MMCPPULL_T ulliDO                      )   ;
    ELMO_INT16  ECATIOReadAnalogInput       (ELMO_UINT8 ucIndex                     )   ;
    void        ECATIOWriteAnalogOutput     (ELMO_UINT8 ucIndex, ELMO_INT16 sAOValue)   ;

};

#endif /* CMMCECATIO_H_ */
