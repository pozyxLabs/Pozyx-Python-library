import requests
import json
import pypozyx
import warnings
import time


class Version(object):
    def __init__(self, version_string):
        self.version = version_string
        if version_string.startswith("v"):
            self.version = version_string[1:]

        if self.version.count('.') == 2:
            self.major, self.minor, self.bugfix = self.version.split('.')
        else:
            self.major, self.minor, self.bugfix = self.version.split('.') + ['0']

    def __eq__(self, other):
        return self.major == other.major and self.minor == other.minor and self.bugfix == other.bugfix

    def __gt__(self, other):
        if self.major > other.major:
            return True
        elif self.major < other.major:
            return False
        else:
            if self.minor > other.minor:
                return True
            elif self.minor < other.minor:
                return False
            else:
                if self.bugfix > other.bugfix:
                    return True
                elif self.bugfix < other.bugfix:
                    return False
        return False

    def __ge__(self, other):
        if self.major > other.major:
            return True
        elif self.major < other.major:
            return False
        else:
            if self.minor > other.minor:
                return True
            elif self.minor < other.minor:
                return False
            else:
                if self.bugfix > other.bugfix:
                    return True
                elif self.bugfix < other.bugfix:
                    return False
        return True

    def __str__(self):
        return "{}.{}.{}".format(self.major, self.minor, self.bugfix)

    def __repr__(self):
        return str(self)


def is_on_latest_version():
    response = requests.get('https://api.github.com/repos/pozyxLabs/Pozyx-Python-library/releases')

    releases = json.loads(response.content.decode())

    versions = [Version(release["tag_name"]) for release in releases]

    highest_release = max(versions)

    pypozyx_version = Version(pypozyx.version)

    if pypozyx_version >= highest_release:
        return True, pypozyx_version
    else:
        return False, highest_release


def perform_latest_version_check():
    try:
        on_latest_version, latest_version = is_on_latest_version()
        if on_latest_version:
            print("Using the latest PyPozyx version {}\n".format(latest_version))
        else:
            warnings.warn("New PyPozyx version available, please upgrade to {}\n".format(latest_version), stacklevel=2)
    except Exception as e:
        warnings.warn("Could not get latest release version from GitHub, {}".format(e), stacklevel=2)


if __name__ == '__main__':
    print(is_on_latest_version())
