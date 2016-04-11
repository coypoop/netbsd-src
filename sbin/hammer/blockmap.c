/*
 * Copyright (c) 2008 The DragonFly Project.  All rights reserved.
 *
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $DragonFly: src/sbin/hammer/blockmap.c,v 1.2 2008/06/17 04:03:38 dillon Exp $
 */

#include "hammer.h"

/*
 * Allocate big-blocks using our poor-man's volume->vol_free_off.
 *
 * If the zone is HAMMER_ZONE_FREEMAP_INDEX we are bootstrapping the freemap
 * itself and cannot update it yet.
 */
hammer_off_t
alloc_bigblock(struct volume_info *volume, int zone)
{
	struct volume_info *root_vol;
	hammer_blockmap_t freemap;
	struct buffer_info *buffer1 = NULL;
	struct buffer_info *buffer2 = NULL;
	struct hammer_blockmap_layer1 *layer1;
	struct hammer_blockmap_layer2 *layer2;
	hammer_off_t layer1_offset;
	hammer_off_t layer2_offset;
	hammer_off_t result_offset;

	if (volume == NULL)
		volume = get_root_volume();

	result_offset = volume->vol_free_off;
	if (result_offset >= volume->vol_free_end)
		errx(1, "alloc_bigblock: Ran out of room, filesystem too small");

	volume->vol_free_off += HAMMER_BIGBLOCK_SIZE;

	/*
	 * Update the freemap if not zone4.
	 */
	if (zone != HAMMER_ZONE_FREEMAP_INDEX) {
		root_vol = get_root_volume();
		freemap = &root_vol->ondisk->vol0_blockmap[HAMMER_ZONE_FREEMAP_INDEX];

		layer1_offset = freemap->phys_offset +
			        HAMMER_BLOCKMAP_LAYER1_OFFSET(result_offset);
		layer1 = get_buffer_data(layer1_offset, &buffer1, 0);
		assert(layer1->phys_offset != HAMMER_BLOCKMAP_UNAVAIL);
		--layer1->blocks_free;
		layer1->layer1_crc = crc32(layer1, HAMMER_LAYER1_CRCSIZE);
		buffer1->cache.modified = 1;

		layer2_offset = layer1->phys_offset +
			        HAMMER_BLOCKMAP_LAYER2_OFFSET(result_offset);
		layer2 = get_buffer_data(layer2_offset, &buffer2, 0);
		assert(layer2->zone == 0);
		layer2->zone = zone;
		layer2->append_off = HAMMER_BIGBLOCK_SIZE;
		layer2->bytes_free = 0;
		layer2->entry_crc = crc32(layer2, HAMMER_LAYER2_CRCSIZE);
		buffer2->cache.modified = 1;

		--root_vol->ondisk->vol0_stat_freebigblocks;

		rel_buffer(buffer1);
		rel_buffer(buffer2);
		rel_volume(root_vol);
	}

	rel_volume(volume);
	return(result_offset);
}

/*
 * Allocate a chunk of data out of a blockmap.  This is a simplified
 * version which uses next_offset as a simple allocation iterator.
 */
void *
alloc_blockmap(int zone, int bytes, hammer_off_t *result_offp,
	       struct buffer_info **bufferp)
{
	struct volume_info *volume;
	hammer_blockmap_t blockmap;
	hammer_blockmap_t freemap;
	struct buffer_info *buffer1 = NULL;
	struct buffer_info *buffer2 = NULL;
	struct hammer_blockmap_layer1 *layer1;
	struct hammer_blockmap_layer2 *layer2;
	hammer_off_t layer1_offset;
	hammer_off_t layer2_offset;
	hammer_off_t chunk_offset;
	void *ptr;

	volume = get_root_volume();

	blockmap = &volume->ondisk->vol0_blockmap[zone];
	freemap = &volume->ondisk->vol0_blockmap[HAMMER_ZONE_FREEMAP_INDEX];
	assert(HAMMER_ZONE_DECODE(blockmap->next_offset) == zone);

	/*
	 * Alignment and buffer-boundary issues.  If the allocation would
	 * cross a buffer boundary we have to skip to the next buffer.
	 */
	bytes = (bytes + 15) & ~15;
	assert(bytes > 0 && bytes <= HAMMER_BUFSIZE);  /* not HAMMER_XBUFSIZE */
	assert(hammer_is_zone2_mapped_index(zone));

again:
	assert(blockmap->next_offset != HAMMER_ZONE_ENCODE(zone + 1, 0));

	if ((blockmap->next_offset ^ (blockmap->next_offset + bytes - 1)) &
	    ~HAMMER_BUFMASK64) {
		blockmap->next_offset = (blockmap->next_offset + bytes - 1) &
				        ~HAMMER_BUFMASK64;
	}
	chunk_offset = blockmap->next_offset & HAMMER_BIGBLOCK_MASK;

	/*
	 * Dive layer 1.
	 */
	layer1_offset = freemap->phys_offset +
			HAMMER_BLOCKMAP_LAYER1_OFFSET(blockmap->next_offset);
	layer1 = get_buffer_data(layer1_offset, &buffer1, 0);
	assert(!(chunk_offset == 0 && layer1->blocks_free == 0));

	if (layer1->phys_offset == HAMMER_BLOCKMAP_UNAVAIL) {
		fprintf(stderr, "alloc_blockmap: ran out of space!\n");
		exit(1);
	}

	/*
	 * Dive layer 2, each entry represents a big-block.
	 */
	layer2_offset = layer1->phys_offset +
			HAMMER_BLOCKMAP_LAYER2_OFFSET(blockmap->next_offset);
	layer2 = get_buffer_data(layer2_offset, &buffer2, 0);

	if (layer2->zone == HAMMER_ZONE_UNAVAIL_INDEX) {
		fprintf(stderr, "alloc_blockmap: ran out of space!\n");
		exit(1);
	}

	/*
	 * If we are entering a new big-block assign ownership to our
	 * zone.  If the big-block is owned by another zone skip it.
	 */
	if (layer2->zone == 0) {
		--layer1->blocks_free;
		layer1->layer1_crc = crc32(layer1, HAMMER_LAYER1_CRCSIZE);
		layer2->zone = zone;
		--volume->ondisk->vol0_stat_freebigblocks;
		assert(layer2->bytes_free == HAMMER_BIGBLOCK_SIZE);
		assert(layer2->append_off == 0);
	}
	if (layer2->zone != zone) {
		blockmap->next_offset = (blockmap->next_offset + HAMMER_BIGBLOCK_SIZE) &
					~HAMMER_BIGBLOCK_MASK64;
		goto again;
	}

	assert(layer2->append_off == chunk_offset);
	layer2->bytes_free -= bytes;
	*result_offp = blockmap->next_offset;
	blockmap->next_offset += bytes;
	layer2->append_off = (int)blockmap->next_offset & HAMMER_BIGBLOCK_MASK;
	layer2->entry_crc = crc32(layer2, HAMMER_LAYER2_CRCSIZE);

	ptr = get_buffer_data(*result_offp, bufferp, 0);
	(*bufferp)->cache.modified = 1;

	buffer1->cache.modified = 1;
	buffer2->cache.modified = 1;

	rel_buffer(buffer1);
	rel_buffer(buffer2);
	rel_volume(volume);
	return(ptr);
}

hammer_off_t
blockmap_lookup(hammer_off_t zone_offset,
		struct hammer_blockmap_layer1 *save_layer1,
		struct hammer_blockmap_layer2 *save_layer2,
		int *errorp)
{
	struct volume_info *root_volume = NULL;
	hammer_blockmap_t blockmap;
	hammer_blockmap_t freemap;
	struct hammer_blockmap_layer1 *layer1;
	struct hammer_blockmap_layer2 *layer2;
	struct buffer_info *buffer1 = NULL;
	struct buffer_info *buffer2 = NULL;
	hammer_off_t layer1_offset;
	hammer_off_t layer2_offset;
	hammer_off_t result_offset;
	int zone;
	int i;
	int error = 0;

	if (save_layer1)
		bzero(save_layer1, sizeof(*save_layer1));
	if (save_layer2)
		bzero(save_layer2, sizeof(*save_layer2));

	zone = HAMMER_ZONE_DECODE(zone_offset);

	if (zone <= HAMMER_ZONE_RAW_VOLUME_INDEX)
		error = -1;
	if (zone >= HAMMER_MAX_ZONES)
		error = -2;
	if (error) {
		result_offset = HAMMER_OFF_BAD;
		goto done;
	}

	root_volume = get_root_volume();
	blockmap = &root_volume->ondisk->vol0_blockmap[zone];

	if (zone == HAMMER_ZONE_RAW_BUFFER_INDEX) {
		result_offset = zone_offset;
	} else if (zone == HAMMER_ZONE_UNDO_INDEX) {
		i = (zone_offset & HAMMER_OFF_SHORT_MASK) /
		    HAMMER_BIGBLOCK_SIZE;
		if (zone_offset >= blockmap->alloc_offset) {
			error = -3;
			result_offset = HAMMER_OFF_BAD;
			goto done;
		}
		result_offset = root_volume->ondisk->vol0_undo_array[i] +
				(zone_offset & HAMMER_BIGBLOCK_MASK64);
	} else {
		result_offset = hammer_xlate_to_zone2(zone_offset);
	}

	/*
	 * The blockmap should match the requested zone (else the volume
	 * header is mashed).
	 *
	 * Note that a valid offset can still be returned if AssertOnFailure
	 * is zero.
	 */
	if (HAMMER_ZONE_FREEMAP_INDEX != zone &&
	    HAMMER_ZONE_DECODE(blockmap->alloc_offset) != zone) {
		error = -4;
		goto done;
	}

	/*
	 * Validate that the big-block is assigned to the zone.  Also
	 * assign save_layer{1,2}.
	 */

	freemap = &root_volume->ondisk->vol0_blockmap[HAMMER_ZONE_FREEMAP_INDEX];
	/*
	 * Dive layer 1.
	 */
	layer1_offset = freemap->phys_offset +
			HAMMER_BLOCKMAP_LAYER1_OFFSET(result_offset);
	layer1 = get_buffer_data(layer1_offset, &buffer1, 0);
	if (layer1 == NULL) {
		error = -5;
		goto done;
	}
	if (layer1->phys_offset == HAMMER_BLOCKMAP_UNAVAIL) {
		error = -6;
		goto done;
	}

	if (save_layer1)
		*save_layer1 = *layer1;

	/*
	 * Dive layer 2, each entry represents a big-block.
	 */
	layer2_offset = layer1->phys_offset +
			HAMMER_BLOCKMAP_LAYER2_OFFSET(result_offset);
	layer2 = get_buffer_data(layer2_offset, &buffer2, 0);

	if (layer2 == NULL) {
		error = -7;
		goto done;
	}
	if (layer2->zone != zone) {
		error = -8;
		goto done;
	}
	if (save_layer2)
		*save_layer2 = *layer2;

done:
	rel_buffer(buffer1);
	rel_buffer(buffer2);
	rel_volume(root_volume);

	if (AssertOnFailure && error != 0)
		errx(1, "blockmap_lookup: error=%d\n", error);
	if (errorp)
		*errorp = error;

	return(result_offset);
}

