/*
 * Geardrive - Sega Mega Drive / Genesis Emulator
 * Copyright (C) 2014  Ignacio Sanchez Gines

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ 
 * 
 */

#ifndef GD_AUDIO_H_
#define	GD_AUDIO_H_

#include "definitions.h"
//#include "audio/Multi_Buffer.h"

//class Sms_Apu;
//class Sound_Queue;

class Audio
{
public:
    Audio();
    ~Audio();
    void Init();
    void Reset();
    void Enable(bool enabled);
    bool IsEnabled() const;
    void SetSampleRate(int rate);
    void WriteAudioRegister(u8 value);
    void WriteGGStereoRegister(u8 value);
    void EndFrame();
    void Tick(unsigned int clock_cycles);

private:
    bool enabled_;
//    Sms_Apu* m_pApu;
//    Stereo_Buffer* m_pBuffer;
//    long m_Time;
//    Sound_Queue* m_pSound;
//    int m_iSampleRate;
//    blip_sample_t* m_pSampleBuffer;
};

const long kSampleBufferSize = 8192;

#endif // GD_AUDIO_H_
