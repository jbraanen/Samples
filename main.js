class Vector {
  constructor() {
    this.X = 0;
    this.Y = 0;

    this.Index = 0;
  }

  Clone() {
    return Vector.FromXY(this.X, this.Y);
  }

  Abs() {
    return Vector.FromXY(Math.abs(this.X), Math.abs(this.Y));
  }

  Equals(B) {
    return this.X === B.X && this.Y === B.Y;
  }

  static FromAB(A, B) {
    let diff = Vector.Minus(B, A);
    return diff;
  }

  static FromXY(X, Y) {
    let v = new Vector();
    v.X = X; v.Y = Y;
    return v;
  }

  static FromVector(Other) {
    let v = new Vector();
    v.SetXY(Other.X, Other.Y);
    return v;
  }

  static Mult(v, n) {
    let v2 = Vector.FromVector(v);
    v2.SetXY(v2.X * n, v2.Y * n);
    return v2;
  }



  static Plus(A, B) {
    let v = new Vector();
    let xx = A.X + B.X;
    let yy = A.Y + B.Y;
    v.SetXY(xx, yy);
    return v;
  }

  static Minus(A, B) {
    let v = new Vector();
    let xx = A.X - B.X;
    let yy = A.Y - B.Y;
    v.SetXY(xx, yy);
    return v;
  }

  Length() {
    let l = Math.sqrt(this.X * this.X + this.Y * this.Y);
    return l;
  }

  Unit() {
    let l = this.Length();
    let XX = this.X / l;
    let YY = this.Y / l;

    return Vector.FromXY(XX, YY);
  }

  Round() {
    return Vector.FromXY(Math.round(this.X), Math.round(this.Y));
  }

  Floor() {
    return Vector.FromXY(Math.floor(this.X), Math.floor(this.Y));
  }

  SetXY(XX, YY) {
    this.X = XX; this.Y = YY;
  }

  Print() {
    console.error("v:");
  }

  Dist(other) {
    let dx = other.X - this.X;
    let dy = other.Y - this.Y;
    let d = Math.sqrt(dx * dx + dy * dy);
    return d;
  }

  static Cross(A, B) {
    return (A.X * B.Y) - (A.Y * B.X);
  }

  static Dot(A, B) {
    return (A.X * B.X) + (A.Y * B.Y);
  }

  //public static Vector2 PerpendicularClockwise(this Vector2 vector2) {
  //    return new Vector2(vector2.Y, -vector2.X);
  //}

  //public static Vector2 PerpendicularCounterClockwise(this Vector2 vector2) {
  //    return new Vector2(-vector2.Y, vector2.X);
  //}

  //Clockwise
  static Orthogonal(v) {
    let ortho = Vector.FromVector(v);
    ortho.X = v.Y;
    ortho.Y = -v.X;
    return ortho;
  }

}

class Polygon {
  constructor() {
    this.vert = [];
  }

  Add(v) {
    //console.error("adding v", v);
    v.Index = this.vert.length;
    this.vert.push(v);
  }

  //Todo LINQ
  //From doesn't need to be a point in the polygon
  MostDistant(from) {
    let farthestQ = this.vert[0];
    let farthestDistance = from.Dist(farthestQ);

    for (let i = 1; i < this.vert.length; i++) {
      let Q = this.vert[i];
      let dist = from.Dist(Q);
      if (dist > farthestDistance) {
        farthestDistance = dist;
        farthestQ = Q;
      }
    }
    return farthestQ;
  }

  Closest(from) {
    let closestQ = this.vert[0];
    let closestDistance = from.Dist(closestQ);

    for (let i = 1; i < this.vert.length; i++) {
      let Q = this.vert[i];
      let dist = from.Dist(Q);
      if (dist < closestDistance) {
        closestDistance = dist;
        closestQ = Q;
      }
    }
    return closestQ;

  }
}


class Sampler {
  constructor() {
    this.history = [];
  }

  IsClose(X, X1) {
    let diff = Math.abs(X - X1);
    return diff < 4;
  }

  Add(X, Y) {
    let historyPoint = Vector.FromXY(X, Y).Round();
    this.history.push(historyPoint);
  }

  Last() {
    return this.history[this.history.length - 1];
  }
  Previous() {
    return this.history[this.history.length - 2];
  }

  Any(X, Y) {
    let present = this.history.some(x => x.X == X && x.Y == Y);
    return present;
  }
}


class ShadowsOfTheNight {

  GetWarmerOrColder(bombDir, point, P0, P1) {
    let P = new Polygon();
    P.Add(P0);
    P.Add(P1);

    if (bombDir == "SAME") {




      return "SAME";
    }
    else if (bombDir == "WARMER") {
      let closest = P.Closest(point);
      return closest;
    }
    else {
      let mostDistant = P.MostDistant(point);
      return mostDistant;
    }
  }

  warmestX(P0, P1, sampler, Left, Right, Top, Bottom, turnsLeft) {
    let usable = sampler.Last();
    let H = Vector.Mult(Vector.Plus(P0, P1), 0.5);
    let diff = Vector.Minus(H, usable).Abs();
    if (diff.X == 0)       //We had an unfortunate last sample, continue with next possibble
      return null;
    let mirror = usable.Clone();
    if (usable.X > H.X) {
      mirror.X = H.X - diff.X;
    }
    else {
      mirror.X = H.X + diff.X;
    }

    if (mirror.X < 0 || mirror.X >= this.Width) {
      if (mirror.X < 0) {
        mirror.X = 0;
        let newH = (usable.X + mirror.X) / 2;
        if (newH >= Right)
          return null;
      }
      else {
        mirror.X = this.Width - 1;
        let newH = (usable.X + mirror.X) / 2;
        if (newH <= Left)
          return null;
      }
    }

    this.sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft);
    let bombDir = this.getBombDir(mirror.X, mirror.Y, sampler, false);
    let result = this.GetWarmerOrColder(bombDir, mirror, P0, P1);
    return result;
  }

  warmestY(P0, P1, sampler, Left, Right, Top, Bottom, turnsLeft) {
    let usable = sampler.Last();
    let H = Vector.Mult(Vector.Plus(P0, P1), 0.5);
    let diff = Vector.Minus(H, usable).Abs();
    if (diff.Y == 0) {
      return null; //Unfortunate H, the same as previosu sample
    }
    let mirror = usable.Clone();
    if (usable.Y > H.Y) {
      mirror.Y = H.Y - diff.Y;
    }
    else {
      mirror.Y = H.Y + diff.Y;
    }

    if (mirror.Y < 0 || mirror.Y >= this.Height) {
      if (mirror.Y < 0) {
        mirror.Y = 0;
        let newH = (usable.Y + mirror.Y) / 2;
        if (newH >= Bottom)
          return null;
      }
      else {
        mirror.Y = this.Height - 1;
        let newH = (usable.Y + mirror.Y) / 2;
        if (newH <= Top)
          return null;
      }
    }
    this.sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft);
    let bombDir = this.getBombDir(mirror.X, mirror.Y, sampler);
    let result = this.GetWarmerOrColder(bombDir, mirror, P0, P1);
    return result;
  }

  IsLegalAndNotPrevious(X, Y, sampler) {
    let legal = X >= 0 && X < this.Height && Y >= 0 && Y < this.Height && !sampler.Any(X, Y);
    return legal;

  }

  getBombDir(X, Y, sampler) {
    console.assert(X == Math.floor(X));
    console.assert(Y == Math.floor(Y));
    console.assert(X >= 0);
    console.assert(Y >= 0);
    console.assert(X < Global.Width);
    console.assert(Y < Global.Height);

    if (X < 0 || Y < 0 || X >= Global.Width || Y >= Global.Height)
      throw "Invalid X or Y, X:" + X + " Y:" + Y;

    //If one turn left, make it count




    //Don't sample the same window
    if (sampler.Any(X, Y)) {
      let New = Vector.FromXY(X, Y);
      let Last = sampler.Last();
      let distance = Last.Dist(New);
      if (distance > 10) {
        //Jumped a bit, so it doesn't matter a lot if we move around just a little bit
        let NewX1 = X - 1;
        let NewX2 = X + 1;
        let NewY1 = Y - 1;
        let NewY2 = Y + 1;

        if (this.IsLegalAndNotPrevious(X, NewY1, sampler)) {
          Y = NewY1;
        }
        else if (this.IsLegalAndNotPrevious(X, NewY2, sampler)) {
          Y = NewY2;
        }
        else if (this.IsLegalAndNotPrevious(NewX1, Y, sampler)) {
          X = NewX1;
        }
        else if (this.IsLegalAndNotPrevious(NewX2, Y, sampler)) {
          X = NewX2;
        }
        else if (this.IsLegalAndNotPrevious(NewX1, NewY1, sampler)) {
          X = NewX1;
          Y = NewY1;
        }
        else if (this.IsLegalAndNotPrevious(NewX2, NewY2, sampler)) {
          X = NewX2;
          Y = NewY2;
        }
        else if (this.IsLegalAndNotPrevious(NewX1, NewY2, sampler)) {
          X = NewX1;
          Y = NewY2;
        }
        else if (this.IsLegalAndNotPrevious(NewX2, NewY1, sampler)) {
          X = NewX2;
          Y = NewY1;
        }
      }
    }

    sampler.Add(X, Y);
    let batPos = Vector.FromXY(X, Y);
    console.log(batPos.X, batPos.Y);

    this.SampleTurn += 1;
    console.error('turn', this.SampleTurn);

    if (this.simulate) {
      if (this.bombPos.Equals(batPos)) {
        this.Finished = this.SampleTurn;
        console.error('Sampled the bomb position on turn', this.SampleTurn);
        throw "FoundTheBomb";
      }
    }

    let BombDir = 'UNDEFINED';

    if (this.simulate) {
      let batmanPos = Vector.FromXY(X, Y);
      let dist = batmanPos.Dist(this.bombPos);
      if (sampler.history.length <= 1) {
        BombDir = "UNDEFINED";
      }
      else {
        let prev = sampler.history[sampler.history.length - 2];
        let prevDist = prev.Dist(this.bombPos);
        if (dist < prevDist)
          BombDir = 'WARMER';
        else if (prevDist < dist)
          BombDir = 'COLDER';
        else
          BombDir = 'SAME';
      }
    } else {
      BombDir = this.readLine();
    }
    console.error(BombDir);
    return BombDir;
  }

  AdjustLeftOrRight(Left, Right, batX, bombDir) {
    let Half = Left + (Right - Left) / 2;

    if (bombDir == "WARMER") {
      //Last sample (batX) is better than the previous sample
      //move the furthest limit towards the bat
      if (batX < Half) {
        //bat is on the left side
        Half = Left + Math.floor((Right - Left) / 2); //floor: we want right to move all down to left
        Right = Half;
      }
      else {
        let Half = Left + Math.ceil((Right - Left) / 2); //ceil: we want right to move all up to left
        Left = Half;
      }
    } else {
      if (batX < Half) {
        //bat is on the left side
        Half = Left + Math.ceil((Right - Left) / 2);
        Left = Half;
      }
      else {
        Half = Left + Math.floor((Right - Left) / 2); //floor: we want right to move all down to left
        Right = Half;
      }
    }

    let result = { Left: Left, Right: Right };
    return result;
  }

  AdjustTopOrBottom(Top, Bottom, batY, bombDir) {
    let Half = Top + (Bottom - Top) / 2;

    if (bombDir == "WARMER") {
      //Last sample (batX) is better than the previous sample
      //move the furthest limit towards the bat
      if (batY < Half) {
        //bat is on the Top side
        Half = Top + Math.floor((Bottom - Top) / 2); //floor: we want Bottom to move all down to Top
        Bottom = Half;
      }
      else {
        let Half = Top + Math.ceil((Bottom - Top) / 2); //ceil: we want Bottom to move all up to Top
        Top = Half;
      }
    } else {
      if (batY < Half) {
        //bat is on the Top side
        Half = Top + Math.ceil((Bottom - Top) / 2);
        Top = Half;
      }
      else {
        Half = Top + Math.floor((Bottom - Top) / 2); //floor: we want Bottom to move all down to Top
        Bottom = Half;
      }
    }

    let result = { Top: Top, Bottom: Bottom };
    return result;
  }

  readLine() {
    // eslint-disable-next-line no-eval
    let str = eval("readline()");
    return str;
  }

  init(Width, Height, bombX, bombY, batX, batY) {
    let result = {};

    if (this.simulate) {
      this.Finished = false;
      this.bombPos = Vector.FromXY(bombX, bombY);
      result.Width = Width;
      result.Height = Height;
      result.batX = batX;
      result.batY = batY;
      let N = 1;
      if (Height == 100)
        N = 12;
      else if (Height == 15)
        N = 12;
      else if (Height == 24)
        N = 15;
      else if (Height == 50)
        N = 16;
      else if (Height == 1000)
        N = 27;
      else if (Height == 8000)
        N = 31;
      result.N = N;
    }
    else {
      var inputs = this.readLine().split(' ');
      result.Width = parseInt(inputs[0]); // width of the building.
      result.Height = parseInt(inputs[1]); // height of the building.
      result.N = parseInt(this.readLine());

      inputs = this.readLine().split(' ');
      result.batX = parseInt(inputs[0]);
      result.batY = parseInt(inputs[1]);

    }
    console.error("Width", result.Width, "Height", result.Height, "batX", result.batX, "batY", result.batY);
    return result;
  }

  FreezeGuard(iter) {
    iter += 1;
    if (iter > 100) {
      throw "TOO MANY ITERATIONS";
    }
    return iter;
  }

  GetTurnsLeft(sampler) {
    let turnsLeft = this.N - sampler.history.length;
    return turnsLeft;
  }

  //Let say we have sampled (batX,batY) last time, now we want to
  //  1. Compute H = halfway from left to right
  //  2. Create mirror point mirrorBatX from batX over H
  //  That way we only need to sample once to get the next information we need

  runGame(Width, Height, bombX, bombY, batX, batY) {
    let initial = this.init(Width, Height, bombX, bombY, batX, batY);
    this.Width = initial.Width;
    this.Height = initial.Height;
    this.N = initial.N;
    batX = initial.batX;
    batY = initial.batY;
    console.error("intial", initial);

    Global.Width = this.Width;
    Global.Height = this.Height;

    this.SampleTurn = 0;
    let sampler = new Sampler();
    if (!this.simulate) {
      this.readLine();
    }
    sampler.Add(batX, batY);
    console.error("sampler", sampler);

    let Top = 0, Left = 0, iter = 0;
    let Right = this.Width - 1;
    let Bottom = this.Height - 1;


    try {
      console.error("1 Left", Left, "Right", Right, "Top", Top, "Bottom", Bottom, "iter", iter);

      let turnsLeft = 10000;
      while (Left < Right || Top < Bottom) {
        console.error("2 Left", Left, "Right", Right, "Top", Top, "Bottom", Bottom, "iter", iter);

        turnsLeft = this.GetTurnsLeft(sampler);
        console.error("turnsleft", turnsLeft);

        this.sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft);


        let canSampleMoreX = true;
        while (Left < Right && canSampleMoreX) {
          batY = sampler.Last().Y;
          let LeftPoint = Vector.FromXY(Left, batY);
          let RightPoint = Vector.FromXY(Right, batY);
          turnsLeft = this.GetTurnsLeft(sampler);
          let warmest = this.warmestX(LeftPoint, RightPoint, sampler, Left, Right, Top, Bottom, turnsLeft);
          if (warmest == null) {
            canSampleMoreX = false;
            continue;
          }
          else if (warmest == "SAME" && LeftPoint.Y == RightPoint.Y) {
            Right = (Left + Right) / 2;
            Left = Right;
          }
          else {
            let S0 = sampler.Previous();
            let S1 = sampler.Last();
            let H = (Math.min(S0.X, S1.X) + Math.max(S0.X, S1.X)) / 2;
            if (warmest.Equals(LeftPoint)) {
              Right = Math.floor(H);
              if (Top == Bottom) {  //Only one dimension
                if (sampler.Any(Left, Top))
                  Left += 1;
              }
            }
            else {
              Left = Math.ceil(H);
              if (Top == Bottom) {  //Only one dimension
                if (sampler.Any(Right, Top))
                  Right -= 1;
              }
            }
          }
          iter = this.FreezeGuard(iter); if (iter > 100) return;
          console.error("Left", Left, "Right", Right, "SampleTurn", this.SampleTurn);
        }
        console.error("Out of X:" + Left);


        let canSampleMoreY = true;
        while (Top < Bottom && canSampleMoreY) {
          batX = sampler.Last().X;
          let TopPoint = Vector.FromXY(batX, Top);
          let BottomPoint = Vector.FromXY(batX, Bottom);
          turnsLeft = this.GetTurnsLeft(sampler);
          let warmest = this.warmestY(TopPoint, BottomPoint, sampler, Left, Right, Top, Bottom, turnsLeft);
          if (warmest == null) {
            canSampleMoreY = false;
            continue;
          }
          else if (warmest == "SAME" && TopPoint.X == BottomPoint.X) {
            Top = (Bottom + Top) / 2;
            Bottom = Top;
          }
          else {
            let S0 = sampler.Previous();
            let S1 = sampler.Last();
            let H = (Math.min(S0.Y, S1.Y) + Math.max(S0.Y, S1.Y)) / 2;
            if (warmest.Equals(TopPoint)) {
              Bottom = Math.floor(H);
              if (Left == Right) {  //Only one dimension
                if (sampler.Any(Left, Top))
                  Top += 1;
              }
            }
            else {
              Top = Math.ceil(H);
              if (Left == Right) {  //Only one dimension
                if (sampler.Any(Left, Bottom))
                  Bottom -= 1;
              }

            }
          }
          console.error("Top", Top, "Bottom", Bottom, "SampleTurn", this.SampleTurn);
          iter = this.FreezeGuard(iter); if (iter > 100) return;
        }
        console.error("Out of Y" + Top);

        //Now, we need two samples.
        //We will pick two in the middle of the rectangle
        //So its easier to reuse on both sides
        let LeftTop = Vector.FromXY(Left, Top);
        let RightBottom = Vector.FromXY(Right, Bottom);
        let P = new Polygon();
        P.Add(LeftTop);
        P.Add(RightBottom);

        let Half = Vector.Mult(Vector.Plus(LeftTop, RightBottom), 0.5).Round();
        // one window on each side of a median
        // that way we might  be able to eliminate more, and also get the "SAME" status if we are lucky
        let LeftAboveH = Vector.Minus(Half, Vector.FromXY(1, 1));
        if (LeftAboveH.X < 0)
          LeftAboveH.X = 0;
        if (LeftAboveH.X >= this.Width)
          LeftAboveH.X = this.Width - 1;
        if (LeftAboveH.Y < 0)
          LeftAboveH.Y = 0;
        if (LeftAboveH.Y >= this.Height)
          LeftAboveH.Y = this.Height - 1;

        batX = LeftAboveH.X;
        batY = LeftAboveH.Y;
        turnsLeft = this.GetTurnsLeft(sampler);
        this.sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft);
        this.getBombDir(batX, batY, sampler, true);
        iter = this.FreezeGuard(iter); if (iter > 100) return;
      }//outer while

      //Make sure we sample this
      turnsLeft = this.GetTurnsLeft(sampler);
      this.sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft);
      this.getBombDir(Left, Top, sampler);

    }//try
    catch (err) {
      console.error("caught: ", err);
      if (err == "FoundTheBomb") {
        return this.SampleTurn;
      }
    }

    ///return this.SampleTurn;
    return 300;
  }//runGame


  sampleSingleIfPossible(Left, Right, Top, Bottom, sampler, turnsLeft) {

    console.error("sampleSingleIfPossible, turnsLeft:", turnsLeft);
    if (turnsLeft == 1) {
      //Make sure to sample one of the windows that are left to sample, and not outside.
      console.error("One turn left, lets sample close to the Left,Top", Left, Top);
      for (let i = Left; i <= Right; i++) {
        for (let j = Top; j <= Bottom; j++) {
          if (sampler.Any(i, j)) {
            console.error("Already sampled", i, j);
            continue;
          }
          else {
            console.log(i, j);
          }
        }
      }
    }

    let windowsLeft = (Right - Left + 1) * (Bottom - Top + 1);
    if (windowsLeft < sampler.history.length) {
      for (let i = Left; i <= Right; i++) {
        for (let j = Top; j <= Bottom; j++) {
          let sampled = sampler.Any(i, j);
          if (sampled)
            windowsLeft -= 1;
        }
      }

      if (windowsLeft <= turnsLeft) {
        //sample all windows we haven't sampled
        for (let i = Left; i <= Right; i++) {
          for (let j = Top; j <= Bottom; j++) {
            let sampled = sampler.Any(i, j);
            if (!sampled) {
              this.getBombDir(i, j, sampler);
            }
          }
        }
      }
    }
  }
}//ShadowOfTheNight


let sotn = new ShadowsOfTheNight();
sotn.simulate=false;
let turns = sotn.runGame();



