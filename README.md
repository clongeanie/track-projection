# Position projetée et interpolation par la distance

## Préliminaire

### Modèle

Les extraits de code donnés ci-après utilisent la classe Trackpoint, dont voici une structure simplifiée.

```typescript
export class Trackpoint {
  location: Location;
  distanceFromStart: number = -1;
  posElevationFromStart: number = -1;
}
```
La classe Location référencée par Trackpoint se définit comme suit.

```typescript
export class Location {
  lat: number;
  lon: number;
  alt?: number;

  constructor(lat: number, lon: number, alt?: number) {
    this.lat = lat;
    this.lon = lon;
    if (typeof alt !== 'undefined') this.alt = alt;
  }
}
```

L'utilisation d'un Quad Tree étant probable, certaines fonctions utilisent une classe QuadTreeSegment,
qui représente simplement les deux points d'un segment.

```typescript
export class QuadTreeSegment {

  index: number;
  from: Trackpoint;
  to: Trackpoint;

  length: number = 0;

  constructor(index: number, from: Trackpoint, to: Trackpoint) {
    this.index = index;
    this.from = from;
    this.to = to;
  }

  get bounds(): Bounds {
    return new Bounds([this.from.location, this.to.location]);
  }

  get averageElevation(): number {
    return (this.from.location.alt! + this.to.location.alt!) / 2;
  }
}
```

### Calcul de la distance entre 2 points

```typescript
  /**
   * Calculates the distance between two points on the globe using the Haversine formula.
   * @param lat1
   * @param lon1
   * @param lat2
   * @param lon2
   * @returns the distance in kilometers
   */
  export function distanceBetween(lat1: number, lon1: number, lat2: number, lon2: number) {
    const R = 6371; // km
    const x1 = lat2 - lat1;
    const dLat = toRadians(x1);
    const x2 = lon2 - lon1;
    const dLon = toRadians(x2);
    const tmp = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(tmp), Math.sqrt(1 - tmp));
    return R * c;
  }
```

## Méthode

### Identifier les segments les plus proches

Pour une position P et un ensemble de segments N, on évalue la distance entre P et chaque segment de N.
Pour un grand nombre de segments, utiliser un Quad Tree en premier lieu afin de réduire le nombre de
segments à évaluer.

On retiendra les segments dont la distance au point est inférieure à un seuil propre à l'application.
S'il en résulte plusieurs segments, on pourra indiquer dans la résultat que la projection n'est pas fiable, ou utiliser 
d'autres données éventuellement à disposition pour discriminer davantage.

> La fonction GCDistanceHaversine utilisée ci-dessous est équilavente à la fonction distanceBetween 
> donnée en préliminaire.

```typescript
function distanceToSegment(p: Trackpoint, start: Trackpoint, end: Trackpoint) {
    const planet = Space.Planet.EARTH;

    const pLat = p.location.lat, pLon = p.location.lon,
      startLat = start.location.lat, startLon = start.location.lon,
      endLat = end.location.lat, endLon = end.location.lon;

    if (startLat === endLat && startLon === endLon) {
      return planet.GCDistanceHaversine(endLat, endLon, pLat, pLon);
    }

    const s0lat = toRadians(pLat);
    const s0lng = toRadians(pLon);
    const s1lat = toRadians(startLat);
    const s1lng = toRadians(startLon);
    const s2lat = toRadians(endLat);
    const s2lng = toRadians(endLon);

    const s2s1lat = s2lat - s1lat;
    const s2s1lng = s2lng - s1lng;
    const u = ((s0lat - s1lat) * s2s1lat + (s0lng - s1lng) * s2s1lng)
      / (s2s1lat * s2s1lat + s2s1lng * s2s1lng);
    if (u <= 0) {
      return planet.GCDistanceHaversine(pLat, pLon, startLat, startLon);
    }
    if (u >= 1) {
      return planet.GCDistanceHaversine(pLat, pLon, endLat, endLon);
    }
    const sa = {
      lat: pLat - startLat,
      lon: pLon - startLon
    };
    const sb = {
      lat: u * (endLat - startLat),
      lon: u * (endLon - startLon)
    };
    return planet.GCDistanceHaversine(sa.lat, sa.lon, sb.lat, sb.lon);
  }
```

### Calculer la projection sur un segment

Une fois les segments les plus proches identifiés, on peut calculer la projection du point sur ceux-ci.

```typescript
function getProjectionOnSegment(location: Location, segment: QuadTreeSegment) {
    if (segment.length === 0) {
      return segment.from;
    }

    const s0lat = toRadians(location.lat);
    const s0lng = toRadians(location.lon);
    const s1lat = toRadians(segment.from.location.lat);
    const s1lng = toRadians(segment.from.location.lon);
    const s2lat = toRadians(segment.to.location.lat);
    const s2lng = toRadians(segment.to.location.lon);

    const s2s1lat = s2lat - s1lat;
    const s2s1lng = s2lng - s1lng;
    const u = ((s0lat - s1lat) * s2s1lat + (s0lng - s1lng) * s2s1lng)
      / (s2s1lat * s2s1lat + s2s1lng * s2s1lng);

    if (u <= 0) return segment.from;
    if (u >= 1) return segment.to;

    let Dx = segment.from.location.lat + u * (segment.to.location.lat - segment.from.location.lat);
    let Dy = segment.from.location.lon + u * (segment.to.location.lon - segment.from.location.lon);
    const result = new Trackpoint();
    result.location = new Location(Dx, Dy);
    if (segment.from.location.alt != null && segment.to.location.alt != null) {
      result.location.alt = (segment.from.location.alt! + segment.to.location.alt!) / 2;
    }
    return result;
  }
```

Alternativement, on peut aussi utiliser une fonction qui combine les deux précédentes. 

```typescript
/**
 * Computes the projection of a point on a segment and the distance between them.
 * @param location
 * @param segment
 * @returns The projection along with the distance of the point to the segment, in kilometers.
 */
  export function getProjectionWithDistance(location: Location, segment: QuadTreeSegment): TrackProjection {
    if (segment.length === 0) {
      // return distance from point to one of the segment's ends
      return {
        trackpoint: segment.from,
        distance: distanceBetween(location.lat, location.lon, segment.from.location.lat, segment.from.location.lon)
      };
    }
    const projection = getProjectionOnSegment(location, segment);
    const dist = distanceBetween(location.lat, location.lon, projection.location.lat, projection.location.lon);
    return {
      trackpoint: projection,
      distance: dist
    };
  }
```

## Interpoler à partir d'une distance parcourue

*Hypothèse : à partir d'une position projetée connue, on souhaite déterminer une nouvelle position à partir de la*
*distance parcourue depuis ce point.*

Pour ce faire, il faut pré-calculer la distance au départ de chaque point d'un tracé. On itère donc sur tous les points de la trace, on calcule la distance au point précédent, on cumule le résultat et on le stocke sur le point courant.

Ensuite, la dernière position GPS connue projetée sur le parcours nous donne la distance depuis le départ. On y ajoute donc la distance parcourue fournie par le système, ce qui nous donne le segment théorique où doit se trouver le sujet.
On peut alors interpoler une position géographique sur le parcours.

```typescript
/**
   * Returns the LatLng which lies the given fraction of the way between the
   * origin LatLng and the destination LatLng.
   *
   * @param from     The LatLng from which to start.
   * @param to       The LatLng toward which to travel.
   * @param fraction A fraction of the distance to travel.
   * @return The interpolated LatLng.
   */
  export function interpolate(from: any, to: any, fraction: number): Location {
    // https://en.wikipedia.org/wiki/Slerp
    const fromLat = toRadians(from.latitude);
    const fromLng = toRadians(from.longitude);
    const toLat = toRadians(to.latitude);
    const toLng = toRadians(to.longitude);
    const cosFromLat = Math.cos(fromLat);
    const cosToLat = Math.cos(toLat);

    // Computes Spherical interpolation coefficients.
    const angle = computeAngleBetween(from, to);
    const sinAngle = Math.sin(angle);
    if (sinAngle < 1E-6) {
      return new Location(
        from.latitude + fraction * (to.latitude - from.latitude),
        from.longitude + fraction * (to.longitude - from.longitude));
    }
    const a = Math.sin((1 - fraction) * angle) / sinAngle;
    const b = Math.sin(fraction * angle) / sinAngle;

    // Converts from polar to vector and interpolate.
    const x = a * cosFromLat * Math.cos(fromLng) + b * cosToLat * Math.cos(toLng);
    const y = a * cosFromLat * Math.sin(fromLng) + b * cosToLat * Math.sin(toLng);
    const z = a * Math.sin(fromLat) + b * Math.sin(toLat);

    // Converts interpolated vector back to polar.
    const lat = Math.atan2(z, Math.sqrt(x * x + y * y));
    const lng = Math.atan2(y, x);
    return new Location(toDegrees(lat), toDegrees(lng));
  }
```
