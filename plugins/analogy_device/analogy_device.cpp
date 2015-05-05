/*
 * Copyright (C) 2012 University of Bristol, UK
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 2 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <analogy_device.h>
#include <debug.h>
#include <math.h>
#include <sstream>

/* For read operation, we consider the default subdevice index is 0 (AI) */
#define ID_SUBD 0
/* For simplicity sake, a maximum channel count is defined */
#define MAX_NB_CHAN 32
/* One hundred triggered scans by default */
#define NB_SCAN 100
/* Default name of analogy */
#define FILENAME "analogy0"
/* Default buffer size */
#define BUF_SIZE 100000

static unsigned char buf[BUF_SIZE];
static char *filename = FILENAME;
static char *str_chans = "1,2";
static unsigned int chans[MAX_NB_CHAN];
static int use_mmap = 1;
static unsigned long wake_count = 0;

/* The command to send by default */
a4l_cmd_t cmd = {
	.idx_subd = ID_SUBD,
	.flags = 0,
	.start_src = TRIG_NOW,
	.start_arg = 0,
	.scan_begin_src = TRIG_TIMER,
	.scan_begin_arg = 8000000,	/* in ns */
	.convert_src = TRIG_TIMER,
	.convert_arg = 500000,	/* in ns */
	.scan_end_src = TRIG_COUNT,
	.scan_end_arg = 0,
	.stop_src = TRIG_COUNT,
	.stop_arg = NB_SCAN,
	.nb_chan = 0,
	.chan_descs = chans,
};

using namespace DAQ;

int dump_text(a4l_desc_t *dsc, a4l_cmd_t *cmd, unsigned char *buf, int size)
{
	static int cur_chan;

	int i, err = 0, tmp_size = 0;
	char *fmts[MAX_NB_CHAN];
	a4l_chinfo_t *chans[MAX_NB_CHAN];

	for (i = 0; i < cmd->nb_chan; i++) {
		int width;

		err = a4l_get_chinfo(dsc,
				cmd->idx_subd,
				cmd->chan_descs[i], &chans[i]);
		if (err < 0) {
			fprintf(stderr,
					"cmd_read: a4l_get_chinfo failed (ret=%d)\n",
					err);
			goto out;
		}

		width = a4l_sizeof_chan(chans[i]);
		if (width < 0) {
			fprintf(stderr,
					"cmd_read: incoherent info for channel %d\n",
					cmd->chan_descs[i]);
			err = width;
			goto out;
		}

		switch(width) {
			case 1:
				fmts[i] = "0x%02x ";
				break;
			case 2:
				fmts[i] = "0x%04x ";
				break;
			case 4:
			default:
				fmts[i] = "0x%08x ";
				break;
		}
	}

	while (tmp_size < size) {
		unsigned long value;

		err = a4l_rawtoul(chans[cur_chan], &value, buf + tmp_size, 1);
		if (err < 0)
			goto out;

		fprintf(stdout, fmts[cur_chan], value);

		/* We assume a4l_sizeof_chan() cannot return because
			 we already called it on the very same channel
			 descriptor */
		tmp_size += a4l_sizeof_chan(chans[cur_chan]);

		if(++cur_chan == cmd->nb_chan) {
			fprintf(stdout, "\n");
			cur_chan = 0;
		}
	}

	fflush(stdout);

out:
	return err;
}

AnalogyDevice::AnalogyDevice(a4l_desc_t *d,std::string name,IO::channel_t *chan,size_t size) : DAQ::Device(name,chan,size), dsc(*d) {

	int err = 0;
	a4l_sbinfo_t *sbinfo;
	a4l_chinfo_t *chinfo;

	// We need to find each subdevice index manually since idx_*_subd often fails
	// Go over all subdevices and save the indexes of the first AI, AO and DIO
	int idx_ai  = -1; 
	int idx_ao  = -1; 
	int idx_dio = -1; 
	for (int i=0; i < dsc.nb_subd; i++) {
		err = a4l_get_subdinfo(&dsc, i, &sbinfo); 
		if(err != 0) {
			ERROR_MSG("AnalogyDriver: a4l_get_subd_info failed, wrong subdevice index %i (err=%d)\n",  i, err);
		}
		// Assign subdevice index; save just the first device if many
		if (((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AI) && (idx_ai < 0))
			idx_ai  = i; 
		else if (((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AO) && (idx_ao < 0))
			idx_ao  = i; 
		else if (((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_DIO) && (idx_dio < 0)) {
			idx_dio  = i; 
		}
	}

	// Get info about AI subdevice
	err = a4l_get_subdinfo(&dsc, idx_ai, &sbinfo);
	if((err == 0) && ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AI)) {
		subdevice[AI].id = idx_ai;
		subdevice[AI].active = 0;
		subdevice[AI].count = sbinfo->nb_chan;
		subdevice[AI].chan = new channel_t[subdevice[AI].count];
		if(!subdevice[AI].chan)
			subdevice[AI].count = 0;
		else
			for(size_t i=0;i<subdevice[AI].count;++i) {
				err = a4l_get_chinfo(&dsc, idx_ai, i, &chinfo);
				// Something went wrong
				if(err < 0) {
					subdevice[AI].active = 0;
					subdevice[AI].count = 0;
					delete[] subdevice[AI].chan;
					break;
				}
				subdevice[AI].chan[i].active = false;
				subdevice[AI].chan[i].analog.maxdata = (1<<chinfo->nb_bits)-1; 
				setAnalogGain(AI,i,1.0);
				setAnalogRange(AI,i,0);
				setAnalogZeroOffset(AI,i,0);
				setAnalogReference(AI,i,0);
				setAnalogUnits(AI,i,0);
			}
	} else {
		subdevice[AI].active = 0;
		subdevice[AI].count = 0;
		subdevice[AI].chan = NULL;
	}

	// Get info about AO subdevice
	err = a4l_get_subdinfo(&dsc, idx_ao, &sbinfo);
	if((err == 0) && ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_AO)) {
		subdevice[AO].id = idx_ao;
		subdevice[AO].active = 0;
		subdevice[AO].count = sbinfo->nb_chan;
		subdevice[AO].chan = new channel_t[subdevice[AO].count];
		if(!subdevice[AO].chan)
			subdevice[AO].count = 0;
		else
			for(size_t i=0;i<subdevice[AO].count;++i) {
				err = a4l_get_chinfo(&dsc, idx_ao, i, &chinfo);
				// Something went wrong
				if(err < 0) {
					subdevice[AO].active = 0;
					subdevice[AO].count = 0;
					delete[] subdevice[AO].chan;
					break;
				}
				subdevice[AO].chan[i].active = false;
				subdevice[AO].chan[i].analog.maxdata = (1<<chinfo->nb_bits)-1;
				setAnalogGain(AO,i,1.0);
				setAnalogZeroOffset(AO,i,0);
				setAnalogRange(AO,i,0);
				setAnalogReference(AO,i,0);
				setAnalogUnits(AO,i,0);
			}
	} else {
		subdevice[AO].active = 0;
		subdevice[AO].count = 0;
		subdevice[AO].chan = NULL;
	}

	// Get info about DIO subdevice and set to INPUT as default
	// Then add all Digital I/O to INPUT list for the
	// IO class to handle
	err = a4l_get_subdinfo(&dsc, idx_dio, &sbinfo);
	if((err == 0) && ((sbinfo->flags & A4L_SUBD_TYPES) == A4L_SUBD_DIO)) {
		subdevice[DIO].id = idx_dio;
		subdevice[DIO].active = 0;
		subdevice[DIO].count = sbinfo->nb_chan;
		subdevice[DIO].chan = new channel_t[subdevice[DIO].count];
		if(!subdevice[DIO].chan)
			subdevice[DIO].count = 0;
		else
			for(size_t i=0;i<subdevice[DIO].count;++i) {
				subdevice[DIO].chan[i].active = false;
				subdevice[DIO].chan[i].digital.previous_value = 0;
				setDigitalDirection(i,DAQ::INPUT);
			}
	} else {
		subdevice[DIO].active = 0;
		subdevice[DIO].count = 0;
		subdevice[DIO].chan = NULL;
	}
	readAsync();
	setActive(true);
}

AnalogyDevice::~AnalogyDevice(void) {
	if(subdevice[AI].chan) delete[] subdevice[AI].chan;
	if(subdevice[AO].chan) delete[] subdevice[AO].chan;
	if(subdevice[DIO].chan) delete[] subdevice[DIO].chan;
	if(dsc.sbdata) a4l_close(&dsc);
}

bool AnalogyDevice::analog_exists(type_t type,index_t count) const {
	if((type == AI && count < subdevice[AI].count) ||
			(type == AO && count < subdevice[AO].count))
		return true;
	return false;
}

// Returns number of channels available for type
size_t AnalogyDevice::getChannelCount(type_t type) const {
	if(type != AI && type != AO && type != DIO)
		return 0;

	return subdevice[type].count;
}

bool AnalogyDevice::getChannelActive(type_t type,index_t channel) const {
	if(channel >= getChannelCount(type))
		return false;

	return subdevice[type].chan[channel].active;    
}

int AnalogyDevice::setChannelActive(type_t type,index_t channel,bool state) {
	if(channel >= getChannelCount(type))
		return -EINVAL;

	if(type == AI)
		output(channel) = 0.0;

	if(subdevice[type].chan[channel].active && !state)
		--subdevice[type].active;
	else if(!subdevice[type].chan[channel].active && state)
		++subdevice[type].active;

	subdevice[type].chan[channel].active = state;
	return 0;
}

size_t AnalogyDevice::getAnalogRangeCount(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0;

	int err = 0;
	a4l_chinfo_t *chinfo;
	a4l_desc_t d = dsc; // Copy descriptior because method is const

	err = a4l_get_chinfo(&d, subdevice[type].id, channel, &chinfo);
	if (err < 0)
		return 0;

	return static_cast<index_t>(chinfo->nb_rng);
}

size_t AnalogyDevice::getAnalogReferenceCount(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0;

	return 4;
}

size_t AnalogyDevice::getAnalogUnitsCount(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0;

	return 2;
}

std::string AnalogyDevice::getAnalogRangeString(type_t type,index_t channel,index_t index) const  {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogRangeCount(type,channel))))
		return "";

	std::ostringstream rangeString;
	int err = 0;
	a4l_rnginfo_t *range;
	a4l_desc_t d = dsc; // Copy descriptior because method is const

	err = a4l_get_rnginfo(&d, subdevice[type].id, channel, index, &range);
	if (err < 0)
		return "";

	rangeString << (double)range->min/1000000 << " to " << (double)range->max/1000000;
	return rangeString.str();
}

std::string AnalogyDevice::getAnalogReferenceString(type_t type,index_t channel,index_t index) const {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogReferenceCount(type,channel))))
		return "";

	switch(index) {
		case 0:
			return "Ground";
		case 1:
			return "Common";
		case 2:
			return "Differential";
		case 3:
			return "Other";
		default:
			return "";
	}
}

std::string AnalogyDevice::getAnalogUnitsString(type_t type,index_t channel,index_t index) const {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogUnitsCount(type,channel))))
		return "";

	switch(index) {
		case 0:
			return "Volts";
		case 1:
			return "Amps";
		default:
			return "";
	}
}

index_t AnalogyDevice::getAnalogRange(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0-1;

	return subdevice[type].chan[channel].analog.range;
}

index_t AnalogyDevice::getAnalogReference(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0-1;

	return subdevice[type].chan[channel].analog.reference;
}

index_t AnalogyDevice::getAnalogUnits(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0-1;

	return subdevice[type].chan[channel].analog.units;
}

index_t AnalogyDevice::getAnalogOffsetUnits(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0-1;

	return subdevice[type].chan[channel].analog.offsetunits;
}

double AnalogyDevice::getAnalogGain(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0;

	return subdevice[type].chan[channel].analog.gain;
}

double AnalogyDevice::getAnalogZeroOffset(type_t type,index_t channel) const {
	if(!analog_exists(type,channel))
		return 0;

	return subdevice[type].chan[channel].analog.zerooffset;
}

int AnalogyDevice::setAnalogRange(type_t type,index_t channel,index_t index)
{
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogRangeCount(type,channel))))
		return -EINVAL;

	channel_t *chan = &subdevice[type].chan[channel];
	int err = 0;
	a4l_rnginfo_t *range;

	err = a4l_get_rnginfo(&dsc, subdevice[type].id, channel, index, &range);
	if (err < 0)
		return -EINVAL;

	chan->analog.range = index;
	chan->analog.conv = (range->max-range->min)/1e6/chan->analog.maxdata;
	chan->analog.offset = -range->min/chan->analog.conv/1e6;

	/*
	 * Save ourselves an extra division each timestep in the fast path.
	 */
	if(type == AO)
		chan->analog.conv = 1/chan->analog.conv;

	return 0;
}

int AnalogyDevice::setAnalogReference(type_t type,index_t channel,index_t index) {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogReferenceCount(type,channel))))
		return -EINVAL;

	subdevice[type].chan[channel].analog.reference = index;
	return 0;
}

int AnalogyDevice::setAnalogUnits(type_t type,index_t channel,index_t index) {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogUnitsCount(type,channel))))
		return -EINVAL;

	subdevice[type].chan[channel].analog.units = index;
	return 0;
}

int AnalogyDevice::setAnalogOffsetUnits(type_t type,index_t channel,index_t index) {
	if(!analog_exists(type,channel) || !((index >= 0) && (index < getAnalogUnitsCount(type,channel))))
		return -EINVAL;

	subdevice[type].chan[channel].analog.offsetunits = index;
	return 0;
}

int AnalogyDevice::setAnalogZeroOffset(type_t type,index_t channel,double offset) {
	if(!analog_exists(type,channel))
		return -EINVAL;

	subdevice[type].chan[channel].analog.zerooffset = offset;
	return 0;
}

int AnalogyDevice::setAnalogGain(type_t type,index_t channel,double gain) {
	if(!analog_exists(type,channel))
		return -EINVAL;

	subdevice[type].chan[channel].analog.gain = gain;
	return 0;
}

// Return the direction of the selected digital channel
direction_t AnalogyDevice::getDigitalDirection(index_t channel) const {
	if(channel >= subdevice[DIO].count)
		return DAQ::INPUT;

	return subdevice[DIO].chan[channel].digital.direction;
}

// Enable digital channel with specific direction
int AnalogyDevice::setDigitalDirection(index_t channel,direction_t direction) {
	if(channel >= subdevice[DIO].count)
		return -EINVAL;

	subdevice[DIO].chan[channel].digital.direction = direction;

	if(direction == DAQ::INPUT)
		return a4l_config_subd(&dsc, subdevice[DIO].id, A4L_INSN_CONFIG_DIO_INPUT, channel);
	else if(direction == DAQ::OUTPUT)
		return a4l_config_subd(&dsc, subdevice[DIO].id, A4L_INSN_CONFIG_DIO_OUTPUT, channel);

	return -EINVAL;
}

// Acquire data - synchronously
void AnalogyDevice::read(void) {
	/*lsampl_t sample;
	analog_channel_t *channel;
	int ref = 0;
	int size = 0;
	int err = -1;
	a4l_chinfo_t *chinfo;

	for(size_t i=0;i < subdevice[AI].count;++i)
		if(subdevice[AI].chan[i].active) {
			channel = &subdevice[AI].chan[i].analog;

			// Get analogy reference 
			switch (channel->reference) {
				case 0: ref = AREF_GROUND; break;
				case 1: ref = AREF_COMMON; break;
				case 2: ref = AREF_DIFF; break;
				case 3: ref = AREF_OTHER; break;
			}

			// Get channel size
			a4l_get_chinfo(&dsc, subdevice[AI].id, i, &chinfo);
			size = a4l_sizeof_chan(chinfo);

			// Read 1 data sample via synchronous acq
			sample = 0;
			err = a4l_sync_read(&dsc, subdevice[AI].id, PACK(i,channel->range,ref),	0, &sample, size);

			output(i) = channel->gain*channel->conv*(sample-channel->offset)+channel->zerooffset;
		}

	unsigned int data = 0, mask = 0xff;
	size_t offset = getChannelCount(AI);

	// Read all data and output it according to channel activity
	a4l_sync_dio(&dsc, subdevice[DIO].id, &mask, &data);
	for(size_t i=0;i < subdevice[DIO].count;++i)
		if(subdevice[DIO].chan[i].active && subdevice[DIO].chan[i].digital.direction == DAQ::INPUT) {
			mask = (1<<i);
			output(i+offset) = (data & mask);
		}*/
}

// Acquire data - asynchronously
void AnalogyDevice::readAsync(void) {
	int ret = 0, len, ofs;
	unsigned int i, scan_size = 0, cnt = 0;
	unsigned long buf_size;
	void *map = NULL;
	//a4l_desc_t dsc = { .sbdata = NULL };

	int (*dump_function) (a4l_desc_t *, a4l_cmd_t*, unsigned char *, int) = dump_text;

	/* Recover the channels to compute */
	do {
		cmd.nb_chan++;
		len = strlen(str_chans);
		ofs = strcspn(str_chans, ",");
		if (sscanf(str_chans, "%u", &chans[cmd.nb_chan - 1]) == 0) {
			fprintf(stderr, "cmd_read: bad channel argument\n");
			//return -EINVAL;
		}
		str_chans += ofs + 1;
	} while (len != ofs);

	/* Update the command structure */
	cmd.scan_end_arg = cmd.nb_chan;
	cmd.stop_src = cmd.stop_arg != 0 ? TRIG_COUNT : TRIG_NONE;

	/* Open the device - already done by driver */
	ret = a4l_open(&dsc, filename);
	if (ret < 0)
	{
		fprintf(stderr, "cmd_read: a4l_open %s failed (ret=%d)\n",filename, ret);
		//return ret;
	}

	printf("cmd_read: device %s opened (fd=%d)\n",filename, dsc.fd);
	printf("cmd_read: basic descriptor retrieved\n");
	printf("\t subdevices count = %d\n", dsc.nb_subd);
	printf("\t read subdevice index = %d\n", dsc.idx_read_subd);
	printf("\t write subdevice index = %d\n", dsc.idx_write_subd);

	/* Allocate a buffer so as to get more info (subd, chan, rng) */
	dsc.sbdata = malloc(dsc.sbsize);
	if (dsc.sbdata == NULL)
	{
		fprintf(stderr, "cmd_read: malloc failed \n");
		//return -ENOMEM;
	}

	/* Get this data - already done by driver */
	ret = a4l_fill_desc(&dsc);
	if (ret < 0)
	{
		fprintf(stderr,"cmd_read: a4l_fill_desc failed (ret=%d)\n", ret);
		goto out_main;
	}

	printf("cmd_read: complex descriptor retrieved\n");

	/* Get the size of a single acquisition */
	for (i = 0; i < cmd.nb_chan; i++)
	{
		a4l_chinfo_t *info;

		ret = a4l_get_chinfo(&dsc, cmd.idx_subd, cmd.chan_descs[i], &info);
		if (ret < 0)
		{
			fprintf(stderr,"cmd_read: a4l_get_chinfo failed (ret=%d)\n",ret);
			goto out_main;
		}

		printf("cmd_read: channel %x\n", cmd.chan_descs[i]);
		printf("\t ranges count = %d\n", info->nb_rng);
		printf("\t bit width = %d (bits)\n", info->nb_bits);
		scan_size += a4l_sizeof_chan(info);
	}

	printf("cmd_read: scan size = %u\n", scan_size);
	if (cmd.stop_arg != 0)
		printf("cmd_read: size to read = %u\n",	scan_size * cmd.stop_arg);

	/* Cancel any former command which might be in progress */
	a4l_snd_cancel(&dsc, cmd.idx_subd);

	ret = a4l_get_bufsize(&dsc, cmd.idx_subd, &buf_size);
	if (ret < 0)
	{
		fprintf(stderr,"cmd_read: a4l_get_bufsize() failed (ret=%d)\n",ret);
		goto out_main;
	}
	printf("cmd_read: buffer size = %lu bytes\n", buf_size);

	/* Map the analog input subdevice buffer */
	ret = a4l_mmap(&dsc, cmd.idx_subd, buf_size, &map);
	if (ret < 0)
	{
		fprintf(stderr,"cmd_read: a4l_mmap() failed (ret=%d)\n",ret);
		goto out_main;
	}
	printf("cmd_read: mmap performed successfully (map=0x%p)\n", map);

	ret = a4l_set_wakesize(&dsc, wake_count);
	if (ret < 0)
	{
		fprintf(stderr,"cmd_read: a4l_set_wakesize failed (ret=%d)\n", ret);
		goto out_main;
	}
	printf("cmd_read: wake size successfully set (%lu)\n", wake_count);

	/* Send the command to the input device */
	ret = a4l_snd_command(&dsc, &cmd);
	if (ret < 0)
	{
		fprintf(stderr,"cmd_read: a4l_snd_command failed (ret=%d)\n", ret);
		goto out_main;
	}
	printf("cmd_read: command successfully sent\n");

	unsigned long front;
	front = 0;

	/* Fetch data without any memcpy */
	do
	{
		/* Retrieve and update the buffer's state
			 (In input case, we recover how many bytes are available
			 to read) */
		ret = a4l_mark_bufrw(&dsc, cmd.idx_subd, front, &front);
		if (ret == -ENOENT)
		{
			break;
		}
		else if (ret < 0)
		{
			fprintf(stderr,"cmd_read: a4l_mark_bufrw() failed (ret=%d)\n",ret);
			goto out_main;
		}

		/* If there is nothing to read, wait for an event
			 (Note that a4l_poll() also retrieves the data amount
			 to read; in our case it is useless as we have to update
			 the data read counter) */
		if (front == 0)
		{
			ret = a4l_poll(&dsc, cmd.idx_subd, A4L_INFINITE);
			if (ret == 0)
				break;
			else if (ret < 0)
			{
				fprintf(stderr,"cmd_read: a4l_poll() failed (ret=%d)\n",ret);
				goto out_main;
			}
		}

		/* Display the results */
		if (dump_function(&dsc,	&cmd,	&((unsigned char *)map)[cnt % buf_size],front) < 0)
		{
			ret = -EIO;
			goto out_main;
		}

		/* Update the counter */
		cnt += front;

	} while (1);
	printf("cmd_read: %d bytes successfully received\n", cnt);
	ret = 0;

out_main:
	/* Clean the pages table */
	munmap(map, buf_size);

	/* Free the buffer used as device descriptor */
	if (dsc.sbdata != NULL)
		free(dsc.sbdata);

	/* Release the file descriptor */
	a4l_close(&dsc);

	//return ret;
}

void AnalogyDevice::write(void) {
	{
		double value;
		lsampl_t sample;
		analog_channel_t *channel;
		int ref = 0;
		int size = 0;
		int err = -1;
		a4l_chinfo_t *chinfo;

		for(size_t i=0;i < subdevice[AO].count;++i)
			if(subdevice[AO].chan[i].active) {
				channel = &subdevice[AO].chan[i].analog;
				value = round(channel->gain*channel->conv*(input(i)-channel->zerooffset)+channel->offset);

				/*
				 * Prevent wrap around in the data units.
				 */
				if(value > channel->maxdata)
					value = channel->maxdata;
				else if(value < 0.0)
					value = 0.0;
				sample = static_cast<lsampl_t>(value);

				// Get anaolgy reference 
				switch (channel->reference) {
					case 0: ref = A4L_CHAN_AREF_GROUND; break;
					case 1: ref = A4L_CHAN_AREF_COMMON; break;
					case 2: ref = A4L_CHAN_AREF_DIFF; break;
					case 3: ref = A4L_CHAN_AREF_OTHER; break;
				}
				// Get channel size
				a4l_get_chinfo(&dsc, subdevice[AO].id, i, &chinfo);
				size = a4l_sizeof_chan(chinfo);

				// Write sample
				err = a4l_sync_write(&dsc, subdevice[AO].id, PACK(i,channel->range,ref),
						0, &sample, size);
			}
	}

	{
		size_t offset = getChannelCount(AO);
		int value;
		int data = 0, mask = 0;

		for(size_t i=0;i < subdevice[DIO].count;++i)
		{
			value = input(i+offset) != 0.0; 
			if(subdevice[DIO].chan[i].active && subdevice[DIO].chan[i].digital.direction == DAQ::OUTPUT && subdevice[DIO].chan[i].digital.previous_value != value)
			{
				subdevice[DIO].chan[i].digital.previous_value = value;
				data ^= (1<<i); // Toggle the i-th bit
				mask |= (1<<i); // Set i-th bit for modification
			}
		}
		// Write the data according to the mask
		if (mask) {
			a4l_sync_dio(&dsc, subdevice[DIO].id, &mask, &data);
		}
	}
}

void AnalogyDevice::doLoad(const Settings::Object::State &s) {
	for(size_t i = 0;i < subdevice[AI].count && i < static_cast<size_t>(s.loadInteger("AI Count"));++i) {
		std::ostringstream str;
		str << i;
		setChannelActive(AI,i,s.loadInteger(str.str()+" AI Active"));
		setAnalogRange(AI,i,s.loadInteger(str.str()+" AI Range"));
		setAnalogReference(AI,i,s.loadInteger(str.str()+" AI Reference"));
		setAnalogUnits(AI,i,s.loadInteger(str.str()+" AI Units"));
		setAnalogGain(AI,i,s.loadDouble(str.str()+" AI Gain"));
		setAnalogZeroOffset(AI,i,s.loadDouble(str.str()+" AI Zero Offset"));
	}

	for(size_t i = 0;i < subdevice[AO].count && i < static_cast<size_t>(s.loadInteger("AO Count"));++i) {
		std::ostringstream str;
		str << i;
		setChannelActive(AO,i,s.loadInteger(str.str()+" AO Active"));
		setAnalogRange(AO,i,s.loadInteger(str.str()+" AO Range"));
		setAnalogReference(AO,i,s.loadInteger(str.str()+" AO Reference"));
		setAnalogUnits(AO,i,s.loadInteger(str.str()+" AO Units"));
		setAnalogGain(AO,i,s.loadDouble(str.str()+" AO Gain"));
		setAnalogZeroOffset(AO,i,s.loadDouble(str.str()+" AO Zero Offset"));
	}

	for(size_t i = 0;i < subdevice[DIO].count && i < static_cast<size_t>(s.loadInteger("DIO Count"));++i) {
		std::ostringstream str;
		str << i;
		setChannelActive(DIO,i,s.loadInteger(str.str()+" DIO Active"));
		setDigitalDirection(i,static_cast<DAQ::direction_t>(s.loadInteger(str.str()+" DIO Direction")));
	}
}

void AnalogyDevice::doSave(Settings::Object::State &s) const {
	s.saveInteger("AI Count",subdevice[AI].count);
	for(size_t i = 0;i < subdevice[AI].count;++i) {
		std::ostringstream str;
		str << i;
		s.saveInteger(str.str()+" AI Active",getChannelActive(AI,i));
		s.saveInteger(str.str()+" AI Range",getAnalogRange(AI,i));
		s.saveInteger(str.str()+" AI Reference",getAnalogReference(AI,i));
		s.saveInteger(str.str()+" AI Units",getAnalogUnits(AI,i));
		s.saveDouble(str.str()+" AI Gain",getAnalogGain(AI,i));
		s.saveDouble(str.str()+" AI Zero Offset",getAnalogZeroOffset(AI,i));
	}

	s.saveInteger("AO Count",subdevice[AO].count);
	for(size_t i = 0;i < subdevice[AO].count;++i) {
		std::ostringstream str;
		str << i;
		s.saveInteger(str.str()+" AO Active",getChannelActive(AO,i));
		s.saveInteger(str.str()+" AO Range",getAnalogRange(AO,i));
		s.saveInteger(str.str()+" AO Reference",getAnalogReference(AO,i));
		s.saveInteger(str.str()+" AO Units",getAnalogUnits(AO,i));
		s.saveDouble(str.str()+" AO Gain",getAnalogGain(AO,i));
		s.saveDouble(str.str()+" AO Zero Offset",getAnalogZeroOffset(AO,i));
	}

	s.saveInteger("DIO Count",subdevice[DIO].count);
	for(size_t i = 0;i < subdevice[DIO].count;++i) {
		std::ostringstream str;
		str << i;
		s.saveInteger(str.str()+" DIO Active",getChannelActive(DIO,i));
		s.saveInteger(str.str()+" DIO Direction",subdevice[DIO].chan[i].digital.direction);
	}
}
