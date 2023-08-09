from os import path, mkdir
from io import BytesIO
import math
from itertools import product
from concurrent.futures import ThreadPoolExecutor, as_completed

import requests
from PIL import Image

class AerialView:
    R = 6378137
    C = 2*math.pi*R # equatorial circumference of the Earth

    def __init__(self, cache_dir="aerialview_cache/", max_retries=3, http_timeout=60, zoom=20, baseurl = None):
        self.cache_dir = cache_dir
        self.max_retries = max_retries
        self.http_timeout = http_timeout
        self.zoom = zoom
        self.tile_size = 256
        self.baseurl = baseurl
        self.session = requests.Session()

        if not path.exists(cache_dir):
            mkdir(cache_dir)


    def getAerialImage(self, lat, lon, bearing, altitude, fov, output_size=(512,512)):
        '''Given a lat,long, bearing angle, altitude and camera FOV, it will return 
        an equivalent aerial image, and the tiles will be cached locally.
        '''
        # size of the square according to altitude and fov
        scamera = self.squareAtCamera(altitude, fov)

        # size that will allow the rotation
        smin = scamera * math.sqrt(2)

        # size of the tile at latitude and zoom level
        szoom = self.squareAtZoomlevel(lat, self.zoom)
        ratio = smin/szoom

        # add extra tiles around the central one to allow
        # rotations and translations
        extra_tiles = math.ceil((math.ceil(ratio)-1)/2)
        extra_tiles = extra_tiles if extra_tiles > 5 else 5

        # tile coordinates from lat/lon/zoom
        xtile, ytile = self._deg2num(lat, lon, self.zoom)

        # current position in pixels
        xcentre = (extra_tiles + (xtile % 1)) * self.tile_size
        ycentre = (extra_tiles + (ytile % 1)) * self.tile_size

        xmin = int(xtile) - extra_tiles
        ymin = int(ytile) - extra_tiles
        xmax = int(xtile) + extra_tiles
        ymax = int(ytile) + extra_tiles

        # all the tile coordinates needed to be recovered
        tiles = tuple(product(range(math.floor(xmin), math.ceil(xmax)), 
                              range(math.floor(ymin), math.ceil(ymax)),))

        futures = []
        with ThreadPoolExecutor(5) as executor:
            for idx, (x, y) in enumerate(tiles):
                futures.append(
                    executor.submit(self._future_wrapper, 
                                    self.baseurl.format(x=x, y=y, z=self.zoom),
                                    idx)
                )
            bbox = (math.floor(xmin), math.floor(ymin), 
                    math.ceil(xmax), math.ceil(ymax))
            final_img = None

            for future in as_completed(futures):
                res, idx = future.result()
                tile = tiles[idx]
                final_img = self._paste_tile(final_img, (self.tile_size, self.tile_size), 
                                             res, tile, bbox)

        scamera_px = scamera/(szoom/self.tile_size)
        d = math.ceil(scamera_px/2)

        crop_box=(xcentre-d, ycentre-d, xcentre+d, ycentre+d)

        if abs(bearing) > 0:
            # rotates counter-clockwise
            final_img = final_img.rotate(angle=bearing, center=(xcentre, ycentre)).crop(crop_box)
        else:
            final_img = final_img.crop(crop_box)

        return final_img.resize(output_size)


    @staticmethod
    def squareAtCamera(altitude, fov):
        return 2*altitude*math.tan(math.radians(fov/2))


    @staticmethod
    def squareAtZoomlevel(lat, z):
        return AerialView.C*math.cos(math.radians(lat))/2**z


    @staticmethod
    def pixelAtZoomlevel(lat, z, tile_size=256):
        return (AerialView.C*math.cos(math.radians(lat))/2**z)/tile_size

    @staticmethod
    def _deg2num(lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = ((lon_deg + 180.0) / 360.0 * n)
        ytile = ((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return (xtile, ytile)

    @staticmethod
    def _num2deg(xtile, ytile, zoom):
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return (lat_deg, lon_deg)


    def _future_wrapper(self, url, idx):
        return self._get_tile(url), idx


    def _get_tile(self, url):
        # check if url is not already cached
        url2filename = "".join([c for c in url if c.isalpha() or c.isdigit() or c==' ']).rstrip()
        tile_path = path.join(self.cache_dir, url2filename + ".png")
        if path.exists(tile_path):
            return Image.open(tile_path)

        retry = self.max_retries
        while True:
            try:
                res = self.session.get(url, timeout=self.http_timeout)
                break
            except Exception:
                retry -= 1
                if not retry:
                    raise

        if (res.status_code == 404) or (not res.content):
            print(f"Error retrieving {url}")
            raise ValueError(f"Empty image received for url {url}")
            
        else:
            res.raise_for_status()
            img = Image.open(BytesIO(res.content))
            img.save(tile_path)
            return img


    def _paste_tile(self, full_img, base_size, img, xy, bbox):
        if img is None:
            return full_img

        mode = "RGB"
        if img.mode != mode:
            img = img.convert(mode)

        size = img.size
        if full_img is None:
            full_img = Image.new(mode, (size[0] * (bbox[2] - bbox[0]), size[1] * (bbox[3] - bbox[1])))
        dx = abs(xy[0] - bbox[0])
        dy = abs(xy[1] - bbox[1])
        xy0 = (size[0] * dx, size[1] * dy)

        full_img.paste(img, xy0)

        return full_img


    @classmethod
    def getBasemaps(free_only=True):
        try:
            import xyzservices.providers as xyz_providers
        except ImportError:
            raise ImportError("You need to install xyzservices: 'pip install xyzservices'")

        basemaps = {}
        providers = xyz_providers.flatten()
        for name, obj in providers.items():
            if free_only and obj.requires_token():
                continue
            basemaps[name] = obj.url

        return basemaps


    @staticmethod
    def getPointAtDistance(lat, lon, d, bearing):
        '''
        lat: initial latitude, in degrees
        lon: initial longitude, in degrees
        d: target distance from initial (m)
        bearing: (true) heading in degrees
        '''
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        a = math.radians(bearing)
        lat2 = math.asin(math.sin(lat1) * math.cos(d/AerialView.R) + math.cos(lat1) * math.sin(d/AerialView.R) * math.cos(a))
        lon2 = lon1 + math.atan2(math.sin(a) * math.sin(d/AerialView.R) * math.cos(lat1),
                               math.cos(d/AerialView.R) - math.sin(lat1) * math.sin(lat2))
        return (math.degrees(lat2), math.degrees(lon2))