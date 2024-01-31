/*
 * CMMCDS401Axis.hpp
 *
 *  Created on: 27/03/2011
 *      Author: yuvall
 *      Update: Haim H. 03Mar2015 Changes for support Multi type of OS.
 */

#ifndef CMMCDS401AXIS_HPP_
#define CMMCDS401AXIS_HPP_

#include "MMCAxis.hpp"
#include "MMCNode.hpp"
#include "MMCPPGlobal.hpp"

class DLLMMCPP_API CMMCDS401Axis : public CMMCNode {
public:
    CMMCDS401Axis();
    ~CMMCDS401Axis();

    void EnableDS401DIChangedEvent() ;
    void DisableDS401DIChangedEvent() ;
    MMCPPULL_T ReadDS401DInput() ;
    void WriteDS401DOutput(MMCPPULL_T) ;
    //
    //
    /*! \fn void ConfigGeneralRPDO3()
    *   \brief This function configure to receive general PDO3 message.
    *   \return - void.
    */
    void ConfigGeneralRPDO3(ELMO_UINT8 ucEventType, ELMO_UINT8 ucPDOCommParam, ELMO_UINT8 ucPDOLength) ;

    //
    /*! \fn void ConfigGeneralRPDO4()
    *   \brief This function configure to receive general PDO4 message.
    *   \return - void.
    */
    void ConfigGeneralRPDO4(ELMO_UINT8 ucEventType, ELMO_UINT8 ucPDOCommParam, ELMO_UINT8 ucPDOLength) ;

    //
    /*! \fn void CancelGeneralRPDO3()
    *   \brief This function cancel the configuration of receiving general PDO3 message.
    *   \return - void.
    */
    void CancelGeneralRPDO3() ;

    //
    /*! \fn void CancelGeneralRPDO4()
    *   \brief This function cancle the configuration of receiving general PDO4 message.
    *   \return - void.
    */
    void CancelGeneralRPDO4() ;

    //
    /*! \fn void ConfigGeneralTPDO3()
    *   \brief This function configure to transmit general PDO3 message.
    *   \return - void.
    */
    void ConfigGeneralTPDO3(ELMO_UINT8 ucEventType) ;

    //
    /*! \fn void ConfigGeneralTPDO4()
    *   \brief This function configure to transmit general PDO4 message.
    *   \return - void.
    */
    void ConfigGeneralTPDO4(ELMO_UINT8 ucEventType) ;

    //
    /*! \fn void CancelGeneralTPDO3()
    *   \brief This function cancle the configuration of transmitting general PDO3 message.
    *   \return - void.
    */
    void CancelGeneralTPDO3() ;

    //
    /*! \fn void CancelGeneralTPDO4()
    *   \brief This function cancle the configuration of transmitting general PDO3 message.
    *   \return - void.
    */
    void CancelGeneralTPDO4() ;

};

#endif /* CMMCDS401AXIS_HPP_ */
