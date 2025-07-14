// This file is part of the NXP NFC Stack
лежит здесь:

NxpNfcRdLib_PN5180_v07.13.00_Pub\NxpNfcRdLib\comps\palI14443p4\src\phpalI14443p4.c
NxpNfcRdLib_PN5180_v07.13.00_Pub\NxpNfcRdLib\comps\palI14443p4\inc\phpalI14443p4.h

структура pDataParams:
typedef struct
{
    void * pDataParams;             /**< Component for which logging shall be performed. */
    phLog_LogEntry_t * pLogEntries; /**< An array of \ref phLog_LogEntry_t structures. */
    uint16_t wNumLogEntries;        /**< Number of valid entries within the array. */
    uint16_t wMaxLogEntries;        /**< Number of log entries the \ref phLog_LogEntry_t array can hold. */
    uint16_t wLogEntriesStart;      /**< If a access occures during the log execution this is required */
} phLog_RegisterEntry_t;



phStatus_t phpalI14443p4_Exchange(
                                  void * pDataParams,
                                  uint16_t wOption,
                                  uint8_t * pTxBuffer,
                                  uint16_t wTxLength,
                                  uint8_t ** ppRxBuffer,
                                  uint16_t * pRxLength
                                  )
{
    phStatus_t PH_MEMLOC_REM status;

    PH_LOG_HELPER_ALLOCATE_TEXT(bFunctionName, "phpalI14443p4_Exchange");
    /*PH_LOG_HELPER_ALLOCATE_PARAMNAME(pDataParams);*/
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(wOption);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(pTxBuffer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(ppRxBuffer);
    PH_LOG_HELPER_ALLOCATE_PARAMNAME(status);
    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_DEBUG, wOption_log, &wOption);
    PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, pTxBuffer_log, pTxBuffer, wTxLength);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_ENTER);
    PH_ASSERT_NULL (pDataParams);
    if (0U != (wTxLength)) PH_ASSERT_NULL (pTxBuffer);

    /* Check data parameters */
    if (PH_GET_COMPCODE(pDataParams) != PH_COMP_PAL_ISO14443P4)
    {
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_PAL_ISO14443P4);

        PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
        PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
        PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

        return status;
    }

    /* perform operation on active layer */
    switch (PH_GET_COMPID(pDataParams))
    {
#ifdef NXPBUILD__PHPAL_I14443P4_SW
    case PHPAL_I14443P4_SW_ID:
        status = phpalI14443p4_Sw_Exchange((phpalI14443p4_Sw_DataParams_t *)pDataParams, wOption, pTxBuffer, wTxLength, ppRxBuffer, pRxLength);
        break;
#endif /* NXPBUILD__PHPAL_I14443P4_SW */

    default:
        status = PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_DATA_PARAMS, PH_COMP_PAL_ISO14443P4);
        break;
    }

    PH_LOG_HELPER_ADDSTRING(PH_LOG_LOGTYPE_INFO, bFunctionName);
#ifdef NXPBUILD__PH_LOG
    if ((((status & PH_ERR_MASK) == PH_ERR_SUCCESS) ||
        ((status & PH_ERR_MASK) == PH_ERR_SUCCESS_CHAINING) ||
        ((status & PH_ERR_MASK) == PH_ERR_SUCCESS_INCOMPLETE_BYTE)) &&
        (0U == ((wOption & PH_EXCHANGE_BUFFERED_BIT))) &&
        (ppRxBuffer != NULL))
    {
        PH_LOG_HELPER_ADDPARAM_BUFFER(PH_LOG_LOGTYPE_DEBUG, ppRxBuffer_log, *ppRxBuffer, *pRxLength);
    }
#endif
    PH_LOG_HELPER_ADDPARAM_UINT16(PH_LOG_LOGTYPE_INFO, status_log, &status);
    PH_LOG_HELPER_EXECUTE(PH_LOG_OPTION_CATEGORY_LEAVE);

    return status;
}


полный код формирования I-Block — откройте файл:

NxpNfcRdLib\comps\palI14443p4\src\Sw\phpalI14443p4_Sw.c

и найдите функцию phpalI14443p4_Sw_Exchange.

phStatus_t phpalI14443p4_Sw_Exchange(
                                     phpalI14443p4_Sw_DataParams_t * pDataParams,
                                     uint16_t wOption,
                                     uint8_t * pTxBuffer,
                                     uint16_t wTxLength,
                                     uint8_t ** ppRxBuffer,
                                     uint16_t * pRxLength
                                     )
{
    phStatus_t  PH_MEMLOC_REM status;
    phStatus_t  PH_MEMLOC_REM statusTmp;
    uint8_t     PH_MEMLOC_REM bBufferOverflow;

    /* Used to build I/R/S block frames */
    uint8_t     PH_MEMLOC_REM bIsoFrame[3];
    uint16_t    PH_MEMLOC_REM wIsoFrameLen = 0;
    uint8_t     PH_MEMLOC_REM bRxOverlapped[3];
    uint16_t    PH_MEMLOC_REM wRxOverlappedLen = 0;
    uint16_t    PH_MEMLOC_REM wRxStartPos;
    uint8_t     PH_MEMLOC_REM bUseNad = 0;
    uint8_t     PH_MEMLOC_REM bForceSend;

    /* Used for Transmission */
    uint16_t    PH_MEMLOC_REM wRxBufferSize;
    uint16_t    PH_MEMLOC_REM wTxBufferSize;
    uint16_t    PH_MEMLOC_REM wTxBufferLen = 0;
    uint16_t    PH_MEMLOC_REM wInfLength = 0;
    uint16_t    PH_MEMLOC_REM wMaxPcdFrameSize;
    uint16_t    PH_MEMLOC_REM wMaxCardFrameSize;
    uint16_t    PH_MEMLOC_REM wPcb = 0;
    uint8_t     PH_MEMLOC_REM bRetryCountRetransmit;

    /* Used for Reception */
    uint16_t    PH_MEMLOC_REM RxLength;
    uint8_t *   PH_MEMLOC_REM pRxBuffer = NULL;

    /* Option parameter check */
    if (0u != (wOption &  (uint16_t)~(uint16_t)
        (
        PH_EXCHANGE_BUFFERED_BIT | PH_EXCHANGE_LEAVE_BUFFER_BIT |
        PH_EXCHANGE_TXCHAINING | PH_EXCHANGE_RXCHAINING | PH_EXCHANGE_RXCHAINING_BUFSIZE
        )))
    {
        return PH_ADD_COMPCODE_FIXED(PH_ERR_INVALID_PARAMETER, PH_COMP_PAL_ISO14443P4);
    }

    /* Check if caller has provided valid RxBuffer */
    if (ppRxBuffer == NULL)
    {
        ppRxBuffer = &pRxBuffer;
    }
    if (pRxLength == NULL)
    {
        pRxLength = &RxLength;
    }

    /* Retrieve HAL buffer sizes */
    PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_GetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_RXBUFFER_BUFSIZE, &wRxBufferSize));
    PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_GetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER_BUFSIZE, &wTxBufferSize));

    /* Retrieve maximum frame sizes */
    wMaxPcdFrameSize = bI14443p4_FsTable[pDataParams->bFsdi] - (uint16_t)2U;
    wMaxCardFrameSize = bI14443p4_FsTable[pDataParams->bFsci] - (uint16_t)2U;

    /* R(ACK) transmission in case of Rx-Chaining */
    if (((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_RXCHAINING) ||
        ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_RXCHAINING_BUFSIZE))
    {
        pDataParams->bStateNow = PHPAL_I14443P4_SW_STATE_I_BLOCK_RX | PHPAL_I14443P4_SW_STATE_CHAINING_BIT;
    }
    /* I-Block transmission */
    else
    {
        /* Reset to default state if not in Tx-Mode */
        if ((pDataParams->bStateNow & PH_EXCHANGE_MODE_MASK) != PHPAL_I14443P4_SW_STATE_I_BLOCK_TX)
        {
            pDataParams->bStateNow = PHPAL_I14443P4_SW_STATE_I_BLOCK_TX;
        }
    }

    /* Reset receive length */
    *pRxLength = 0;

    /* Reset RetryCount */
    bRetryCountRetransmit = 0;

    /* Reset BufferOverflow flag */
    bBufferOverflow = 0;

    /* ******************************** */
    /*     I-BLOCK TRANSMISSION LOOP    */
    /* ******************************** */
    do
    {
        /* Reset Preloaded bytes and ForceSend */
        wTxBufferLen = 0;
        bForceSend = 0;

        switch (pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_MASK)
        {
        case PHPAL_I14443P4_SW_STATE_I_BLOCK_TX:

            /* Retrieve Number of preloaded bytes */
            if (0U != (wOption & PH_EXCHANGE_LEAVE_BUFFER_BIT))
            {
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_GetConfig(
                    pDataParams->pHalDataParams,
                    PHHAL_HW_CONFIG_TXBUFFER_LENGTH,
                    &wTxBufferLen));
            }

            /* Set initial INF length to (remaining) input data length */
            wInfLength = wTxLength;

            /* Frame has already been preloaded -> IsoFrameLen is zero */
            if (wTxBufferLen > 0U)
            {
                /* do not generate the iso frame */
                wIsoFrameLen = 0;
            }
            /* Else evaluate IsoFrameLen*/
            else
            {
                /* 7.1.1.3 c), ISO/IEC 14443-4:2008(E), "During chaining the NAD shall only be transmitted in the first block of chain." */
                if ((0U == ((pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_CHAINING_BIT))))
                {
                    bUseNad = pDataParams->bNadEnabled;
                }
                else
                {
                    bUseNad = 0;
                }

                /* Evaluate frame overhead */
                wIsoFrameLen = 1;
                if (0U != (bUseNad))
                {
                    ++wIsoFrameLen;
                }
                if (0U != (pDataParams->bCidEnabled))
                {
                    ++wIsoFrameLen;
                }
            }

            /* Check if chaining is intended or not */
            if ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_TXCHAINING)
            {
                pDataParams->bStateNow |= PHPAL_I14443P4_SW_STATE_CHAINING_BIT;
            }
            else
            {
                pDataParams->bStateNow &= (uint8_t)~(uint8_t)PHPAL_I14443P4_SW_STATE_CHAINING_BIT;
            }

            /* Force frame exchange if
            a) the maximum frame size of the card has been reached;
            */
            if ((wTxBufferLen + wIsoFrameLen + wInfLength) > wMaxCardFrameSize)
            {
                /* force frame exchange */
                bForceSend = 1;

                /* force chaining */
                pDataParams->bStateNow |= PHPAL_I14443P4_SW_STATE_CHAINING_BIT;

                /* limit number of bytes to send */
                wInfLength = wMaxCardFrameSize - wTxBufferLen - wIsoFrameLen;
            }

            /* Force frame exchange if
            b) the TxBuffer is full;
            */
            if ((0U != ((wOption & PH_EXCHANGE_BUFFERED_BIT))) &&
                ((wTxBufferLen + wIsoFrameLen + wInfLength) >= wTxBufferSize))
            {
                /* force frame exchange */
                bForceSend = 1;

                /* force chaining */
                pDataParams->bStateNow |= PHPAL_I14443P4_SW_STATE_CHAINING_BIT;
            }

            /* Generate / Buffer ISO frame */
            if (wIsoFrameLen > 0U)
            {
                /* Generate I-Block frame header */
                PH_CHECK_SUCCESS_FCT(statusTmp, phpalI14443p4_Sw_BuildIBlock(
                    pDataParams->bCidEnabled,
                    pDataParams->bCid,
                    bUseNad,
                    pDataParams->bNad,
                    pDataParams->bPcbBlockNum,
                    (0U != (pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_CHAINING_BIT)) ? 1U : 0U,
                    bIsoFrame,
                    &wIsoFrameLen));

                /* Write Frame to HAL TxBuffer but do not preform Exchange */
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Exchange(
                    pDataParams->pHalDataParams,
                    PH_EXCHANGE_BUFFER_FIRST,
                    bIsoFrame,
                    wIsoFrameLen,
                    NULL,
                    NULL));

                /* Retain the preloaded bytes from now on */
                wOption |= PH_EXCHANGE_LEAVE_BUFFER_BIT;
            }

            /* Tx-Buffering mode (and no forced exchange) */
            if ((0U == bForceSend) && (0U != (wOption & PH_EXCHANGE_BUFFERED_BIT)))
            {
                /* Preload the data into the TxBuffer */
                return phhalHw_Exchange(
                    pDataParams->pHalDataParams,
                    (wOption & (uint16_t)~(uint16_t)PH_EXCHANGE_MODE_MASK),
                    pTxBuffer,
                    wInfLength,
                    NULL,
                    NULL);
            }

            /* Content has been buffered before */
            if (wTxBufferLen > 0U)
            {
                /* retrieve PCB byte */
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER_OFFSET, 0x00));
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_GetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER, &wPcb));

                /* Preloaded Data or ForceSend -> Modify PCB byte if neccessary */
                if ((0U != bForceSend) || ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_TXCHAINING))
                {
                    /* modify PCB byte */
                    wPcb |= PHPAL_I14443P4_SW_PCB_CHAINING;
                    PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER, wPcb));
                }
            }
            break;

        case PHPAL_I14443P4_SW_STATE_I_BLOCK_RX:

            /* Build R(ACK) frame */
            PH_CHECK_SUCCESS_FCT(statusTmp, phpalI14443p4_Sw_BuildRBlock(
                pDataParams->bCidEnabled,
                pDataParams->bCid,
                pDataParams->bPcbBlockNum,
                1,
                bIsoFrame,
                &wIsoFrameLen));

            /* Write Frame to HAL TxBuffer but do not preform Exchange */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_Exchange(
                pDataParams->pHalDataParams,
                PH_EXCHANGE_BUFFERED_BIT,
                bIsoFrame,
                wIsoFrameLen,
                NULL,
                NULL));

            /* Retain the preloaded bytes from now on */
            wOption |= PH_EXCHANGE_LEAVE_BUFFER_BIT;

            /* do not append any data */
            wInfLength = 0;
            break;

            /* Should NEVER happen! */
        default:
            return PH_ADD_COMPCODE_FIXED(PH_ERR_INTERNAL_ERROR, PH_COMP_PAL_ISO14443P4);
        }

        /* Perform Exchange using complete ISO handling */
        status = phpalI14443p4_Sw_IsoHandling(
            pDataParams,
            wOption & (uint16_t)~(uint16_t)PH_EXCHANGE_BUFFERED_BIT,
            bRetryCountRetransmit,
            pTxBuffer,
            wInfLength,
            ppRxBuffer,
            pRxLength);

        /* Complete chaining if buffer is full */
        if (((status & PH_ERR_MASK) == PH_ERR_BUFFER_OVERFLOW) &&
            ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_RXCHAINING_BUFSIZE))
        {
            /* Indicate Buffer Overflow */
            bBufferOverflow = 1;

            /* Toggle Blocknumber */
            pDataParams->bPcbBlockNum ^= PHPAL_I14443P4_SW_PCB_BLOCKNR;
        }
        /* Else bail out on error */
        else
        {
            PH_CHECK_SUCCESS(status);
        }

        /* Retransmission in progress */
        if (0U != (pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_RETRANSMIT_BIT))
        {
            /* Increment Retransmit RetryCount */
            ++bRetryCountRetransmit;

            /* Restore internal TxBuffer. */
            /* Neccessary in case RxBuffer and TxBuffer are the same. */
            if (wTxBufferLen > 0U)
            {
                /* restore PCB byte */
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER_OFFSET, 0x00));
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_TXBUFFER, wPcb));

                /* restore TxBufferLen */
                PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(
                    pDataParams->pHalDataParams,
                    PHHAL_HW_CONFIG_TXBUFFER_LENGTH,
                    wTxBufferLen));
            }

            /* Clear retransmission bit */
            pDataParams->bStateNow &= (uint8_t)~(uint8_t)PHPAL_I14443P4_SW_STATE_RETRANSMIT_BIT;
        }
        /* No retransmission in progress */
        else
        {
            /* Reset Retransmit RetryCount */
            bRetryCountRetransmit = 0;

            /* Chaining is active */
            if (pDataParams->bStateNow == (PHPAL_I14443P4_SW_STATE_I_BLOCK_TX | PHPAL_I14443P4_SW_STATE_CHAINING_BIT))
            {
                /* Bytes to send cannot be less than sent bytes */
                if (wTxLength < wInfLength)
                {
                    return PH_ADD_COMPCODE_FIXED(PH_ERR_INTERNAL_ERROR, PH_COMP_PAL_ISO14443P4);
                }

                /* Remove sent bytes from TxBuffer */
                pTxBuffer = pTxBuffer + wInfLength;

                /* below if condition is added to make QAC compliant  */
                if(wTxLength > 0U)
                {
                    wTxLength = wTxLength - wInfLength;
                }
            }

            /* Buffered / TxChaining mode -> finished after sending */
            if ((wTxLength == 0U) &&
                (
                ((wOption & PH_EXCHANGE_BUFFERED_BIT) > 0U) ||
                ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_TXCHAINING)
                ))
            {
                return PH_ERR_SUCCESS;
            }
        }
    }
    /* Loop as long as the state does not transit to RX mode */
    while ((pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_MASK) != PHPAL_I14443P4_SW_STATE_I_BLOCK_RX);

    /* Overlap PCB */
    wRxOverlappedLen = 1;

    /* Overlap CID */
    if (0u != ((*ppRxBuffer)[PHPAL_I14443P4_SW_PCB_POS] & PHPAL_I14443P4_SW_PCB_CID_FOLLOWING))
    {
        wRxOverlappedLen++;
    }

    /* Overlap NAD */
    if (0u != ((*ppRxBuffer)[PHPAL_I14443P4_SW_PCB_POS] & PHPAL_I14443P4_SW_PCB_NAD_FOLLOWING))
    {
        wRxOverlappedLen++;
    }

    /* Reset RxStartPos */
    wRxStartPos = 0;

    /* ******************************** */
    /*      I-BLOCK RECEPTION LOOP      */
    /* ******************************** */
    do
    {
        /* Only allow receive state at this point */
        if ((pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_MASK) != PHPAL_I14443P4_SW_STATE_I_BLOCK_RX)
        {
            return PH_ADD_COMPCODE_FIXED(PH_ERR_INTERNAL_ERROR, PH_COMP_PAL_ISO14443P4);
        }

        /* Rule 2, ISO/IEC 14443-4:2008(E), PICC chaining */
        if (0U != (pDataParams->bStateNow & PHPAL_I14443P4_SW_STATE_CHAINING_BIT))
        {
            /* Skip overlapping / SUCCESS_CHAINING checks in case of BufferOverflow */
            if (0U == (bBufferOverflow))
            {
                /* This is first chained response */
                if (wRxStartPos == 0U)
                {
                    /* Special NAD chaining handling */
                    /* 7.1.1.3 c), ISO/IEC 14443-4:2008(E), "During chaining the NAD shall only be transmitted in the first block of chain." */
                    if (0U != (pDataParams->bNadEnabled))
                    {
                        --wRxOverlappedLen;
                    }
                }

                /* Backup overlapped bytes */
                (void)memcpy(bRxOverlapped, &(*ppRxBuffer)[((*pRxLength) - wRxOverlappedLen)], wRxOverlappedLen);

                /* Calculate RxBuffer Start Position */
                wRxStartPos = (*pRxLength) - wRxOverlappedLen;

                /* Skip SUCCESS_CHAINING check for RXCHAINING_BUFSIZE mode */
                if ((wOption & PH_EXCHANGE_MODE_MASK) != PH_EXCHANGE_RXCHAINING_BUFSIZE)
                {
                    /* Return with chaining status if the next chain may not fit into our buffer */
                    if ((*pRxLength + wMaxPcdFrameSize) > wRxBufferSize)
                    {
                        /* Adjust RxBuffer position */
                        (*ppRxBuffer) = (*ppRxBuffer) + wRxOverlappedLen;
                        *pRxLength = *pRxLength - wRxOverlappedLen;

                        return PH_ADD_COMPCODE_FIXED(PH_ERR_SUCCESS_CHAINING, PH_COMP_PAL_ISO14443P4);
                    }
                }
            }

            /* Set RxBuffer Start Position */
            PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(
                pDataParams->pHalDataParams,
                PHHAL_HW_CONFIG_RXBUFFER_STARTPOS,
                wRxStartPos));

            /* Prepare R(ACK) frame */
            PH_CHECK_SUCCESS_FCT(statusTmp, phpalI14443p4_Sw_BuildRBlock(
                pDataParams->bCidEnabled,
                pDataParams->bCid,
                pDataParams->bPcbBlockNum,
                1,
                bIsoFrame,
                &wIsoFrameLen));

            /* Perform Exchange using complete ISO handling */
            status = phpalI14443p4_Sw_IsoHandling(
                pDataParams,
                PH_EXCHANGE_DEFAULT,
                0,
                bIsoFrame,
                wIsoFrameLen,
                ppRxBuffer,
                pRxLength);

            /* Complete chaining if buffer is full */
            if (((status & PH_ERR_MASK) == PH_ERR_BUFFER_OVERFLOW) &&
                ((wOption & PH_EXCHANGE_MODE_MASK) == PH_EXCHANGE_RXCHAINING_BUFSIZE))
            {
                /* Reset wRxStartPos */
                wRxStartPos = 0;

                /* Indicate Buffer Overflow */
                bBufferOverflow = 1;

                /* Toggle Blocknumber */
                pDataParams->bPcbBlockNum ^= PHPAL_I14443P4_SW_PCB_BLOCKNR;
            }
            /* Default behaviour */
            else
            {
                /* In case of buffer overflow error from HAL, reset the HAL Rx Buffer Start position */
                if ((pDataParams->bOpeMode != RD_LIB_MODE_ISO) && ((status & PH_ERR_MASK) == PH_ERR_BUFFER_OVERFLOW))
                {
                    PH_CHECK_SUCCESS_FCT(statusTmp, phhalHw_SetConfig(pDataParams->pHalDataParams, PHHAL_HW_CONFIG_RXBUFFER_STARTPOS, 0));
                }
                /* Bail out on error */
                PH_CHECK_SUCCESS(status);

                /* Restore overlapped INF bytes */
                (void)memcpy(&(*ppRxBuffer)[wRxStartPos], bRxOverlapped, wRxOverlappedLen); /* PRQA S 3354 */
            }
        }
        /* No chaining -> reception finished */
        else
        {
            /* Return data */
            if (0U == (bBufferOverflow))
            {
                /* Special NAD chaining handling */
                /* 7.1.1.3 c), ISO/IEC 14443-4:2008(E), "During chaining the NAD shall only be transmitted in the first block of chain." */
                if ((wRxStartPos > 0U) && (pDataParams->bNadEnabled > 0U))
                {
                    ++wRxOverlappedLen;
                }

                /* Do not return protocol bytes, advance to INF field */
                (*ppRxBuffer) = (*ppRxBuffer) + wRxOverlappedLen;
                *pRxLength = *pRxLength - wRxOverlappedLen;
                /* Reception successful */
                status = PH_ERR_SUCCESS;
            }
            /* do not return any data in case of Buffer Overflow */
            else
            {
                *pRxLength = 0;
                status = PH_ADD_COMPCODE_FIXED(PH_ERR_BUFFER_OVERFLOW, PH_COMP_PAL_ISO14443P4);
            }

            /* Reception finished */
            pDataParams->bStateNow = PHPAL_I14443P4_SW_STATE_FINISHED;
        }
    }
    while (pDataParams->bStateNow != PHPAL_I14443P4_SW_STATE_FINISHED);

    return status;
}