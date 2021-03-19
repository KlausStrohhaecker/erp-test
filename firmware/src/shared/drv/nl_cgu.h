/******************************************************************************/
/** @file		nl_cgu.h
    @date		2015-01-28
    @brief    	CGU functions
    @author		Nemanja Nikodijevic [2015-01-28]
*******************************************************************************/
#pragma once

#define M4_PERIOD_US (125ul)  // M4 ticker interrupt period in 1us multiples
#define M4_FREQ_HZ   (1000000 / M4_PERIOD_US)
