from pypozyx import PozyxConstants, UWBSettings


def all_discovery_uwb_settings():
    for channel in PozyxConstants.ALL_UWB_CHANNELS:
        for bitrate in PozyxConstants.ALL_UWB_BITRATES:
            for prf in PozyxConstants.ALL_UWB_PRFS:
                for plen in [PozyxConstants.UWB_PLEN_64, PozyxConstants.UWB_PLEN_1536]:
                    yield UWBSettings(channel, bitrate, prf, plen, 33)