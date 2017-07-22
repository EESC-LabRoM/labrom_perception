/*************************************************************************
*   Optical flow header files
*   This file is part of labrom_optical_flow
*
*   labrom_optical_flow is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_optical_flow is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_optical_flow.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

// Top-level namespace
namespace optical_flow{
/**
* Erros definition
*/
#define ERROR_NOT_ENOUGH_FEATURES -1
#define ERROR_POOR_TRACKING -2
#define SUCCESS 1


/*
* Operation mode. first bit reserverd to indicate whether output image is required or not
*/
#define MODE_SET_OUTPUT_IMAGE 0x01
#define MODE_SET_ESTIMATE_N   0x02
#define MODE_SET_ESTIMATE_W   0x04
#define MODE_SET_ESTIMATE_V   0x08

} // optical_flow namesace

#endif // OPTICAL_FLOW_H_