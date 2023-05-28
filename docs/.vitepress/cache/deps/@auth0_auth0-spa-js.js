// node_modules/@auth0/auth0-spa-js/dist/auth0-spa-js.production.esm.js
var e = function(t2, n2) {
  return (e = Object.setPrototypeOf || { __proto__: [] } instanceof Array && function(e2, t3) {
    e2.__proto__ = t3;
  } || function(e2, t3) {
    for (var n3 in t3)
      Object.prototype.hasOwnProperty.call(t3, n3) && (e2[n3] = t3[n3]);
  })(t2, n2);
};
function t(t2, n2) {
  if ("function" != typeof n2 && null !== n2)
    throw new TypeError("Class extends value " + String(n2) + " is not a constructor or null");
  function r2() {
    this.constructor = t2;
  }
  e(t2, n2), t2.prototype = null === n2 ? Object.create(n2) : (r2.prototype = n2.prototype, new r2());
}
var n = function() {
  return (n = Object.assign || function(e2) {
    for (var t2, n2 = 1, r2 = arguments.length; n2 < r2; n2++)
      for (var o2 in t2 = arguments[n2])
        Object.prototype.hasOwnProperty.call(t2, o2) && (e2[o2] = t2[o2]);
    return e2;
  }).apply(this, arguments);
};
function r(e2, t2) {
  var n2 = {};
  for (var r2 in e2)
    Object.prototype.hasOwnProperty.call(e2, r2) && t2.indexOf(r2) < 0 && (n2[r2] = e2[r2]);
  if (null != e2 && "function" == typeof Object.getOwnPropertySymbols) {
    var o2 = 0;
    for (r2 = Object.getOwnPropertySymbols(e2); o2 < r2.length; o2++)
      t2.indexOf(r2[o2]) < 0 && Object.prototype.propertyIsEnumerable.call(e2, r2[o2]) && (n2[r2[o2]] = e2[r2[o2]]);
  }
  return n2;
}
function o(e2, t2, n2, r2) {
  return new (n2 || (n2 = Promise))(function(o2, i2) {
    function a2(e3) {
      try {
        s2(r2.next(e3));
      } catch (e4) {
        i2(e4);
      }
    }
    function c2(e3) {
      try {
        s2(r2.throw(e3));
      } catch (e4) {
        i2(e4);
      }
    }
    function s2(e3) {
      var t3;
      e3.done ? o2(e3.value) : (t3 = e3.value, t3 instanceof n2 ? t3 : new n2(function(e4) {
        e4(t3);
      })).then(a2, c2);
    }
    s2((r2 = r2.apply(e2, t2 || [])).next());
  });
}
function i(e2, t2) {
  var n2, r2, o2, i2, a2 = { label: 0, sent: function() {
    if (1 & o2[0])
      throw o2[1];
    return o2[1];
  }, trys: [], ops: [] };
  return i2 = { next: c2(0), throw: c2(1), return: c2(2) }, "function" == typeof Symbol && (i2[Symbol.iterator] = function() {
    return this;
  }), i2;
  function c2(i3) {
    return function(c3) {
      return function(i4) {
        if (n2)
          throw new TypeError("Generator is already executing.");
        for (; a2; )
          try {
            if (n2 = 1, r2 && (o2 = 2 & i4[0] ? r2.return : i4[0] ? r2.throw || ((o2 = r2.return) && o2.call(r2), 0) : r2.next) && !(o2 = o2.call(r2, i4[1])).done)
              return o2;
            switch (r2 = 0, o2 && (i4 = [2 & i4[0], o2.value]), i4[0]) {
              case 0:
              case 1:
                o2 = i4;
                break;
              case 4:
                return a2.label++, { value: i4[1], done: false };
              case 5:
                a2.label++, r2 = i4[1], i4 = [0];
                continue;
              case 7:
                i4 = a2.ops.pop(), a2.trys.pop();
                continue;
              default:
                if (!(o2 = a2.trys, (o2 = o2.length > 0 && o2[o2.length - 1]) || 6 !== i4[0] && 2 !== i4[0])) {
                  a2 = 0;
                  continue;
                }
                if (3 === i4[0] && (!o2 || i4[1] > o2[0] && i4[1] < o2[3])) {
                  a2.label = i4[1];
                  break;
                }
                if (6 === i4[0] && a2.label < o2[1]) {
                  a2.label = o2[1], o2 = i4;
                  break;
                }
                if (o2 && a2.label < o2[2]) {
                  a2.label = o2[2], a2.ops.push(i4);
                  break;
                }
                o2[2] && a2.ops.pop(), a2.trys.pop();
                continue;
            }
            i4 = t2.call(e2, a2);
          } catch (e3) {
            i4 = [6, e3], r2 = 0;
          } finally {
            n2 = o2 = 0;
          }
        if (5 & i4[0])
          throw i4[1];
        return { value: i4[0] ? i4[1] : void 0, done: true };
      }([i3, c3]);
    };
  }
}
function a(e2, t2) {
  var n2 = "function" == typeof Symbol && e2[Symbol.iterator];
  if (!n2)
    return e2;
  var r2, o2, i2 = n2.call(e2), a2 = [];
  try {
    for (; (void 0 === t2 || t2-- > 0) && !(r2 = i2.next()).done; )
      a2.push(r2.value);
  } catch (e3) {
    o2 = { error: e3 };
  } finally {
    try {
      r2 && !r2.done && (n2 = i2.return) && n2.call(i2);
    } finally {
      if (o2)
        throw o2.error;
    }
  }
  return a2;
}
function c(e2, t2, n2) {
  if (n2 || 2 === arguments.length)
    for (var r2, o2 = 0, i2 = t2.length; o2 < i2; o2++)
      !r2 && o2 in t2 || (r2 || (r2 = Array.prototype.slice.call(t2, 0, o2)), r2[o2] = t2[o2]);
  return e2.concat(r2 || t2);
}
var s = "undefined" != typeof globalThis ? globalThis : "undefined" != typeof window ? window : "undefined" != typeof global ? global : "undefined" != typeof self ? self : {};
function u(e2) {
  return e2 && e2.__esModule && Object.prototype.hasOwnProperty.call(e2, "default") ? e2.default : e2;
}
function l(e2, t2) {
  return e2(t2 = { exports: {} }, t2.exports), t2.exports;
}
var f = function(e2) {
  return e2 && e2.Math == Math && e2;
};
var d = f("object" == typeof globalThis && globalThis) || f("object" == typeof window && window) || f("object" == typeof self && self) || f("object" == typeof s && s) || function() {
  return this;
}() || Function("return this")();
var h = function(e2) {
  try {
    return !!e2();
  } catch (e3) {
    return true;
  }
};
var p = !h(function() {
  return 7 != Object.defineProperty({}, 1, { get: function() {
    return 7;
  } })[1];
});
var y = {}.propertyIsEnumerable;
var v = Object.getOwnPropertyDescriptor;
var m = { f: v && !y.call({ 1: 2 }, 1) ? function(e2) {
  var t2 = v(this, e2);
  return !!t2 && t2.enumerable;
} : y };
var g = function(e2, t2) {
  return { enumerable: !(1 & e2), configurable: !(2 & e2), writable: !(4 & e2), value: t2 };
};
var b = {}.toString;
var w = function(e2) {
  return b.call(e2).slice(8, -1);
};
var S = "".split;
var _ = h(function() {
  return !Object("z").propertyIsEnumerable(0);
}) ? function(e2) {
  return "String" == w(e2) ? S.call(e2, "") : Object(e2);
} : Object;
var k = function(e2) {
  if (null == e2)
    throw TypeError("Can't call method on " + e2);
  return e2;
};
var I = function(e2) {
  return _(k(e2));
};
var T = function(e2) {
  return "object" == typeof e2 ? null !== e2 : "function" == typeof e2;
};
var O = function(e2, t2) {
  if (!T(e2))
    return e2;
  var n2, r2;
  if (t2 && "function" == typeof (n2 = e2.toString) && !T(r2 = n2.call(e2)))
    return r2;
  if ("function" == typeof (n2 = e2.valueOf) && !T(r2 = n2.call(e2)))
    return r2;
  if (!t2 && "function" == typeof (n2 = e2.toString) && !T(r2 = n2.call(e2)))
    return r2;
  throw TypeError("Can't convert object to primitive value");
};
var E = function(e2) {
  return Object(k(e2));
};
var x = {}.hasOwnProperty;
var C = Object.hasOwn || function(e2, t2) {
  return x.call(E(e2), t2);
};
var R = d.document;
var L = T(R) && T(R.createElement);
var F = function(e2) {
  return L ? R.createElement(e2) : {};
};
var j = !p && !h(function() {
  return 7 != Object.defineProperty(F("div"), "a", { get: function() {
    return 7;
  } }).a;
});
var U = Object.getOwnPropertyDescriptor;
var K = { f: p ? U : function(e2, t2) {
  if (e2 = I(e2), t2 = O(t2, true), j)
    try {
      return U(e2, t2);
    } catch (e3) {
    }
  if (C(e2, t2))
    return g(!m.f.call(e2, t2), e2[t2]);
} };
var A = function(e2) {
  if (!T(e2))
    throw TypeError(String(e2) + " is not an object");
  return e2;
};
var P = Object.defineProperty;
var W = { f: p ? P : function(e2, t2, n2) {
  if (A(e2), t2 = O(t2, true), A(n2), j)
    try {
      return P(e2, t2, n2);
    } catch (e3) {
    }
  if ("get" in n2 || "set" in n2)
    throw TypeError("Accessors not supported");
  return "value" in n2 && (e2[t2] = n2.value), e2;
} };
var Z = p ? function(e2, t2, n2) {
  return W.f(e2, t2, g(1, n2));
} : function(e2, t2, n2) {
  return e2[t2] = n2, e2;
};
var V = function(e2, t2) {
  try {
    Z(d, e2, t2);
  } catch (n2) {
    d[e2] = t2;
  }
  return t2;
};
var X = d["__core-js_shared__"] || V("__core-js_shared__", {});
var N = Function.toString;
"function" != typeof X.inspectSource && (X.inspectSource = function(e2) {
  return N.call(e2);
});
var D;
var z;
var J;
var Y = X.inspectSource;
var G = d.WeakMap;
var B = "function" == typeof G && /native code/.test(Y(G));
var M = l(function(e2) {
  (e2.exports = function(e3, t2) {
    return X[e3] || (X[e3] = void 0 !== t2 ? t2 : {});
  })("versions", []).push({ version: "3.15.2", mode: "global", copyright: "© 2021 Denis Pushkarev (zloirock.ru)" });
});
var H = 0;
var q = Math.random();
var Q = function(e2) {
  return "Symbol(" + String(void 0 === e2 ? "" : e2) + ")_" + (++H + q).toString(36);
};
var $ = M("keys");
var ee = function(e2) {
  return $[e2] || ($[e2] = Q(e2));
};
var te = {};
var ne = d.WeakMap;
if (B || X.state) {
  re = X.state || (X.state = new ne()), oe = re.get, ie = re.has, ae = re.set;
  D = function(e2, t2) {
    if (ie.call(re, e2))
      throw new TypeError("Object already initialized");
    return t2.facade = e2, ae.call(re, e2, t2), t2;
  }, z = function(e2) {
    return oe.call(re, e2) || {};
  }, J = function(e2) {
    return ie.call(re, e2);
  };
} else {
  ce = ee("state");
  te[ce] = true, D = function(e2, t2) {
    if (C(e2, ce))
      throw new TypeError("Object already initialized");
    return t2.facade = e2, Z(e2, ce, t2), t2;
  }, z = function(e2) {
    return C(e2, ce) ? e2[ce] : {};
  }, J = function(e2) {
    return C(e2, ce);
  };
}
var re;
var oe;
var ie;
var ae;
var ce;
var se;
var ue;
var le = { set: D, get: z, has: J, enforce: function(e2) {
  return J(e2) ? z(e2) : D(e2, {});
}, getterFor: function(e2) {
  return function(t2) {
    var n2;
    if (!T(t2) || (n2 = z(t2)).type !== e2)
      throw TypeError("Incompatible receiver, " + e2 + " required");
    return n2;
  };
} };
var fe = l(function(e2) {
  var t2 = le.get, n2 = le.enforce, r2 = String(String).split("String");
  (e2.exports = function(e3, t3, o2, i2) {
    var a2, c2 = !!i2 && !!i2.unsafe, s2 = !!i2 && !!i2.enumerable, u2 = !!i2 && !!i2.noTargetGet;
    "function" == typeof o2 && ("string" != typeof t3 || C(o2, "name") || Z(o2, "name", t3), (a2 = n2(o2)).source || (a2.source = r2.join("string" == typeof t3 ? t3 : ""))), e3 !== d ? (c2 ? !u2 && e3[t3] && (s2 = true) : delete e3[t3], s2 ? e3[t3] = o2 : Z(e3, t3, o2)) : s2 ? e3[t3] = o2 : V(t3, o2);
  })(Function.prototype, "toString", function() {
    return "function" == typeof this && t2(this).source || Y(this);
  });
});
var de = d;
var he = function(e2) {
  return "function" == typeof e2 ? e2 : void 0;
};
var pe = function(e2, t2) {
  return arguments.length < 2 ? he(de[e2]) || he(d[e2]) : de[e2] && de[e2][t2] || d[e2] && d[e2][t2];
};
var ye = Math.ceil;
var ve = Math.floor;
var me = function(e2) {
  return isNaN(e2 = +e2) ? 0 : (e2 > 0 ? ve : ye)(e2);
};
var ge = Math.min;
var be = function(e2) {
  return e2 > 0 ? ge(me(e2), 9007199254740991) : 0;
};
var we = Math.max;
var Se = Math.min;
var _e = function(e2) {
  return function(t2, n2, r2) {
    var o2, i2 = I(t2), a2 = be(i2.length), c2 = function(e3, t3) {
      var n3 = me(e3);
      return n3 < 0 ? we(n3 + t3, 0) : Se(n3, t3);
    }(r2, a2);
    if (e2 && n2 != n2) {
      for (; a2 > c2; )
        if ((o2 = i2[c2++]) != o2)
          return true;
    } else
      for (; a2 > c2; c2++)
        if ((e2 || c2 in i2) && i2[c2] === n2)
          return e2 || c2 || 0;
    return !e2 && -1;
  };
};
var ke = { includes: _e(true), indexOf: _e(false) };
var Ie = ke.indexOf;
var Te = function(e2, t2) {
  var n2, r2 = I(e2), o2 = 0, i2 = [];
  for (n2 in r2)
    !C(te, n2) && C(r2, n2) && i2.push(n2);
  for (; t2.length > o2; )
    C(r2, n2 = t2[o2++]) && (~Ie(i2, n2) || i2.push(n2));
  return i2;
};
var Oe = ["constructor", "hasOwnProperty", "isPrototypeOf", "propertyIsEnumerable", "toLocaleString", "toString", "valueOf"];
var Ee = Oe.concat("length", "prototype");
var xe = { f: Object.getOwnPropertyNames || function(e2) {
  return Te(e2, Ee);
} };
var Ce = { f: Object.getOwnPropertySymbols };
var Re = pe("Reflect", "ownKeys") || function(e2) {
  var t2 = xe.f(A(e2)), n2 = Ce.f;
  return n2 ? t2.concat(n2(e2)) : t2;
};
var Le = function(e2, t2) {
  for (var n2 = Re(t2), r2 = W.f, o2 = K.f, i2 = 0; i2 < n2.length; i2++) {
    var a2 = n2[i2];
    C(e2, a2) || r2(e2, a2, o2(t2, a2));
  }
};
var Fe = /#|\.prototype\./;
var je = function(e2, t2) {
  var n2 = Ke[Ue(e2)];
  return n2 == Pe || n2 != Ae && ("function" == typeof t2 ? h(t2) : !!t2);
};
var Ue = je.normalize = function(e2) {
  return String(e2).replace(Fe, ".").toLowerCase();
};
var Ke = je.data = {};
var Ae = je.NATIVE = "N";
var Pe = je.POLYFILL = "P";
var We = je;
var Ze = K.f;
var Ve = function(e2, t2) {
  var n2, r2, o2, i2, a2, c2 = e2.target, s2 = e2.global, u2 = e2.stat;
  if (n2 = s2 ? d : u2 ? d[c2] || V(c2, {}) : (d[c2] || {}).prototype)
    for (r2 in t2) {
      if (i2 = t2[r2], o2 = e2.noTargetGet ? (a2 = Ze(n2, r2)) && a2.value : n2[r2], !We(s2 ? r2 : c2 + (u2 ? "." : "#") + r2, e2.forced) && void 0 !== o2) {
        if (typeof i2 == typeof o2)
          continue;
        Le(i2, o2);
      }
      (e2.sham || o2 && o2.sham) && Z(i2, "sham", true), fe(n2, r2, i2, e2);
    }
};
var Xe = pe("navigator", "userAgent") || "";
var Ne = d.process;
var De = Ne && Ne.versions;
var ze = De && De.v8;
ze ? ue = (se = ze.split("."))[0] < 4 ? 1 : se[0] + se[1] : Xe && (!(se = Xe.match(/Edge\/(\d+)/)) || se[1] >= 74) && (se = Xe.match(/Chrome\/(\d+)/)) && (ue = se[1]);
var Je;
var Ye = ue && +ue;
var Ge = !!Object.getOwnPropertySymbols && !h(function() {
  var e2 = Symbol();
  return !String(e2) || !(Object(e2) instanceof Symbol) || !Symbol.sham && Ye && Ye < 41;
});
var Be = Ge && !Symbol.sham && "symbol" == typeof Symbol.iterator;
var Me = M("wks");
var He = d.Symbol;
var qe = Be ? He : He && He.withoutSetter || Q;
var Qe = function(e2) {
  return C(Me, e2) && (Ge || "string" == typeof Me[e2]) || (Ge && C(He, e2) ? Me[e2] = He[e2] : Me[e2] = qe("Symbol." + e2)), Me[e2];
};
var $e = Qe("match");
var et = function(e2) {
  if (function(e3) {
    var t2;
    return T(e3) && (void 0 !== (t2 = e3[$e]) ? !!t2 : "RegExp" == w(e3));
  }(e2))
    throw TypeError("The method doesn't accept regular expressions");
  return e2;
};
var tt = Qe("match");
var nt = function(e2) {
  var t2 = /./;
  try {
    "/./"[e2](t2);
  } catch (n2) {
    try {
      return t2[tt] = false, "/./"[e2](t2);
    } catch (e3) {
    }
  }
  return false;
};
var rt = K.f;
var ot = "".startsWith;
var it = Math.min;
var at = nt("startsWith");
var ct = !(at || (Je = rt(String.prototype, "startsWith"), !Je || Je.writable));
Ve({ target: "String", proto: true, forced: !ct && !at }, { startsWith: function(e2) {
  var t2 = String(k(this));
  et(e2);
  var n2 = be(it(arguments.length > 1 ? arguments[1] : void 0, t2.length)), r2 = String(e2);
  return ot ? ot.call(t2, r2, n2) : t2.slice(n2, n2 + r2.length) === r2;
} });
var st = function(e2) {
  if ("function" != typeof e2)
    throw TypeError(String(e2) + " is not a function");
  return e2;
};
var ut = function(e2, t2, n2) {
  if (st(e2), void 0 === t2)
    return e2;
  switch (n2) {
    case 0:
      return function() {
        return e2.call(t2);
      };
    case 1:
      return function(n3) {
        return e2.call(t2, n3);
      };
    case 2:
      return function(n3, r2) {
        return e2.call(t2, n3, r2);
      };
    case 3:
      return function(n3, r2, o2) {
        return e2.call(t2, n3, r2, o2);
      };
  }
  return function() {
    return e2.apply(t2, arguments);
  };
};
var lt = Function.call;
var ft = function(e2, t2, n2) {
  return ut(lt, d[e2].prototype[t2], n2);
};
ft("String", "startsWith");
var dt = Array.isArray || function(e2) {
  return "Array" == w(e2);
};
var ht = function(e2, t2, n2) {
  var r2 = O(t2);
  r2 in e2 ? W.f(e2, r2, g(0, n2)) : e2[r2] = n2;
};
var pt = Qe("species");
var yt = function(e2, t2) {
  var n2;
  return dt(e2) && ("function" != typeof (n2 = e2.constructor) || n2 !== Array && !dt(n2.prototype) ? T(n2) && null === (n2 = n2[pt]) && (n2 = void 0) : n2 = void 0), new (void 0 === n2 ? Array : n2)(0 === t2 ? 0 : t2);
};
var vt = Qe("species");
var mt = Qe("isConcatSpreadable");
var gt = Ye >= 51 || !h(function() {
  var e2 = [];
  return e2[mt] = false, e2.concat()[0] !== e2;
});
var bt = function(e2) {
  return Ye >= 51 || !h(function() {
    var t2 = [];
    return (t2.constructor = {})[vt] = function() {
      return { foo: 1 };
    }, 1 !== t2[e2](Boolean).foo;
  });
}("concat");
var wt = function(e2) {
  if (!T(e2))
    return false;
  var t2 = e2[mt];
  return void 0 !== t2 ? !!t2 : dt(e2);
};
Ve({ target: "Array", proto: true, forced: !gt || !bt }, { concat: function(e2) {
  var t2, n2, r2, o2, i2, a2 = E(this), c2 = yt(a2, 0), s2 = 0;
  for (t2 = -1, r2 = arguments.length; t2 < r2; t2++)
    if (wt(i2 = -1 === t2 ? a2 : arguments[t2])) {
      if (s2 + (o2 = be(i2.length)) > 9007199254740991)
        throw TypeError("Maximum allowed index exceeded");
      for (n2 = 0; n2 < o2; n2++, s2++)
        n2 in i2 && ht(c2, s2, i2[n2]);
    } else {
      if (s2 >= 9007199254740991)
        throw TypeError("Maximum allowed index exceeded");
      ht(c2, s2++, i2);
    }
  return c2.length = s2, c2;
} });
var St = {};
St[Qe("toStringTag")] = "z";
var _t = "[object z]" === String(St);
var kt = Qe("toStringTag");
var It = "Arguments" == w(function() {
  return arguments;
}());
var Tt = _t ? w : function(e2) {
  var t2, n2, r2;
  return void 0 === e2 ? "Undefined" : null === e2 ? "Null" : "string" == typeof (n2 = function(e3, t3) {
    try {
      return e3[t3];
    } catch (e4) {
    }
  }(t2 = Object(e2), kt)) ? n2 : It ? w(t2) : "Object" == (r2 = w(t2)) && "function" == typeof t2.callee ? "Arguments" : r2;
};
var Ot = _t ? {}.toString : function() {
  return "[object " + Tt(this) + "]";
};
_t || fe(Object.prototype, "toString", Ot, { unsafe: true });
var Et;
var xt = Object.keys || function(e2) {
  return Te(e2, Oe);
};
var Ct = p ? Object.defineProperties : function(e2, t2) {
  A(e2);
  for (var n2, r2 = xt(t2), o2 = r2.length, i2 = 0; o2 > i2; )
    W.f(e2, n2 = r2[i2++], t2[n2]);
  return e2;
};
var Rt = pe("document", "documentElement");
var Lt = ee("IE_PROTO");
var Ft = function() {
};
var jt = function(e2) {
  return "<script>" + e2 + "<\/script>";
};
var Ut = function() {
  try {
    Et = document.domain && new ActiveXObject("htmlfile");
  } catch (e3) {
  }
  var e2, t2;
  Ut = Et ? function(e3) {
    e3.write(jt("")), e3.close();
    var t3 = e3.parentWindow.Object;
    return e3 = null, t3;
  }(Et) : ((t2 = F("iframe")).style.display = "none", Rt.appendChild(t2), t2.src = String("javascript:"), (e2 = t2.contentWindow.document).open(), e2.write(jt("document.F=Object")), e2.close(), e2.F);
  for (var n2 = Oe.length; n2--; )
    delete Ut.prototype[Oe[n2]];
  return Ut();
};
te[Lt] = true;
var Kt = Object.create || function(e2, t2) {
  var n2;
  return null !== e2 ? (Ft.prototype = A(e2), n2 = new Ft(), Ft.prototype = null, n2[Lt] = e2) : n2 = Ut(), void 0 === t2 ? n2 : Ct(n2, t2);
};
var At = xe.f;
var Pt = {}.toString;
var Wt = "object" == typeof window && window && Object.getOwnPropertyNames ? Object.getOwnPropertyNames(window) : [];
var Zt = { f: function(e2) {
  return Wt && "[object Window]" == Pt.call(e2) ? function(e3) {
    try {
      return At(e3);
    } catch (e4) {
      return Wt.slice();
    }
  }(e2) : At(I(e2));
} };
var Vt = { f: Qe };
var Xt = W.f;
var Nt = function(e2) {
  var t2 = de.Symbol || (de.Symbol = {});
  C(t2, e2) || Xt(t2, e2, { value: Vt.f(e2) });
};
var Dt = W.f;
var zt = Qe("toStringTag");
var Jt = function(e2, t2, n2) {
  e2 && !C(e2 = n2 ? e2 : e2.prototype, zt) && Dt(e2, zt, { configurable: true, value: t2 });
};
var Yt = [].push;
var Gt = function(e2) {
  var t2 = 1 == e2, n2 = 2 == e2, r2 = 3 == e2, o2 = 4 == e2, i2 = 6 == e2, a2 = 7 == e2, c2 = 5 == e2 || i2;
  return function(s2, u2, l2, f2) {
    for (var d2, h2, p2 = E(s2), y2 = _(p2), v2 = ut(u2, l2, 3), m2 = be(y2.length), g2 = 0, b2 = f2 || yt, w2 = t2 ? b2(s2, m2) : n2 || a2 ? b2(s2, 0) : void 0; m2 > g2; g2++)
      if ((c2 || g2 in y2) && (h2 = v2(d2 = y2[g2], g2, p2), e2))
        if (t2)
          w2[g2] = h2;
        else if (h2)
          switch (e2) {
            case 3:
              return true;
            case 5:
              return d2;
            case 6:
              return g2;
            case 2:
              Yt.call(w2, d2);
          }
        else
          switch (e2) {
            case 4:
              return false;
            case 7:
              Yt.call(w2, d2);
          }
    return i2 ? -1 : r2 || o2 ? o2 : w2;
  };
};
var Bt = { forEach: Gt(0), map: Gt(1), filter: Gt(2), some: Gt(3), every: Gt(4), find: Gt(5), findIndex: Gt(6), filterOut: Gt(7) }.forEach;
var Mt = ee("hidden");
var Ht = Qe("toPrimitive");
var qt = le.set;
var Qt = le.getterFor("Symbol");
var $t = Object.prototype;
var en = d.Symbol;
var tn = pe("JSON", "stringify");
var nn = K.f;
var rn = W.f;
var on = Zt.f;
var an = m.f;
var cn = M("symbols");
var sn = M("op-symbols");
var un = M("string-to-symbol-registry");
var ln = M("symbol-to-string-registry");
var fn = M("wks");
var dn = d.QObject;
var hn = !dn || !dn.prototype || !dn.prototype.findChild;
var pn = p && h(function() {
  return 7 != Kt(rn({}, "a", { get: function() {
    return rn(this, "a", { value: 7 }).a;
  } })).a;
}) ? function(e2, t2, n2) {
  var r2 = nn($t, t2);
  r2 && delete $t[t2], rn(e2, t2, n2), r2 && e2 !== $t && rn($t, t2, r2);
} : rn;
var yn = function(e2, t2) {
  var n2 = cn[e2] = Kt(en.prototype);
  return qt(n2, { type: "Symbol", tag: e2, description: t2 }), p || (n2.description = t2), n2;
};
var vn = Be ? function(e2) {
  return "symbol" == typeof e2;
} : function(e2) {
  return Object(e2) instanceof en;
};
var mn = function(e2, t2, n2) {
  e2 === $t && mn(sn, t2, n2), A(e2);
  var r2 = O(t2, true);
  return A(n2), C(cn, r2) ? (n2.enumerable ? (C(e2, Mt) && e2[Mt][r2] && (e2[Mt][r2] = false), n2 = Kt(n2, { enumerable: g(0, false) })) : (C(e2, Mt) || rn(e2, Mt, g(1, {})), e2[Mt][r2] = true), pn(e2, r2, n2)) : rn(e2, r2, n2);
};
var gn = function(e2, t2) {
  A(e2);
  var n2 = I(t2), r2 = xt(n2).concat(_n(n2));
  return Bt(r2, function(t3) {
    p && !bn.call(n2, t3) || mn(e2, t3, n2[t3]);
  }), e2;
};
var bn = function(e2) {
  var t2 = O(e2, true), n2 = an.call(this, t2);
  return !(this === $t && C(cn, t2) && !C(sn, t2)) && (!(n2 || !C(this, t2) || !C(cn, t2) || C(this, Mt) && this[Mt][t2]) || n2);
};
var wn = function(e2, t2) {
  var n2 = I(e2), r2 = O(t2, true);
  if (n2 !== $t || !C(cn, r2) || C(sn, r2)) {
    var o2 = nn(n2, r2);
    return !o2 || !C(cn, r2) || C(n2, Mt) && n2[Mt][r2] || (o2.enumerable = true), o2;
  }
};
var Sn = function(e2) {
  var t2 = on(I(e2)), n2 = [];
  return Bt(t2, function(e3) {
    C(cn, e3) || C(te, e3) || n2.push(e3);
  }), n2;
};
var _n = function(e2) {
  var t2 = e2 === $t, n2 = on(t2 ? sn : I(e2)), r2 = [];
  return Bt(n2, function(e3) {
    !C(cn, e3) || t2 && !C($t, e3) || r2.push(cn[e3]);
  }), r2;
};
if (Ge || (fe((en = function() {
  if (this instanceof en)
    throw TypeError("Symbol is not a constructor");
  var e2 = arguments.length && void 0 !== arguments[0] ? String(arguments[0]) : void 0, t2 = Q(e2), n2 = function(e3) {
    this === $t && n2.call(sn, e3), C(this, Mt) && C(this[Mt], t2) && (this[Mt][t2] = false), pn(this, t2, g(1, e3));
  };
  return p && hn && pn($t, t2, { configurable: true, set: n2 }), yn(t2, e2);
}).prototype, "toString", function() {
  return Qt(this).tag;
}), fe(en, "withoutSetter", function(e2) {
  return yn(Q(e2), e2);
}), m.f = bn, W.f = mn, K.f = wn, xe.f = Zt.f = Sn, Ce.f = _n, Vt.f = function(e2) {
  return yn(Qe(e2), e2);
}, p && (rn(en.prototype, "description", { configurable: true, get: function() {
  return Qt(this).description;
} }), fe($t, "propertyIsEnumerable", bn, { unsafe: true }))), Ve({ global: true, wrap: true, forced: !Ge, sham: !Ge }, { Symbol: en }), Bt(xt(fn), function(e2) {
  Nt(e2);
}), Ve({ target: "Symbol", stat: true, forced: !Ge }, { for: function(e2) {
  var t2 = String(e2);
  if (C(un, t2))
    return un[t2];
  var n2 = en(t2);
  return un[t2] = n2, ln[n2] = t2, n2;
}, keyFor: function(e2) {
  if (!vn(e2))
    throw TypeError(e2 + " is not a symbol");
  if (C(ln, e2))
    return ln[e2];
}, useSetter: function() {
  hn = true;
}, useSimple: function() {
  hn = false;
} }), Ve({ target: "Object", stat: true, forced: !Ge, sham: !p }, { create: function(e2, t2) {
  return void 0 === t2 ? Kt(e2) : gn(Kt(e2), t2);
}, defineProperty: mn, defineProperties: gn, getOwnPropertyDescriptor: wn }), Ve({ target: "Object", stat: true, forced: !Ge }, { getOwnPropertyNames: Sn, getOwnPropertySymbols: _n }), Ve({ target: "Object", stat: true, forced: h(function() {
  Ce.f(1);
}) }, { getOwnPropertySymbols: function(e2) {
  return Ce.f(E(e2));
} }), tn) {
  kn = !Ge || h(function() {
    var e2 = en();
    return "[null]" != tn([e2]) || "{}" != tn({ a: e2 }) || "{}" != tn(Object(e2));
  });
  Ve({ target: "JSON", stat: true, forced: kn }, { stringify: function(e2, t2, n2) {
    for (var r2, o2 = [e2], i2 = 1; arguments.length > i2; )
      o2.push(arguments[i2++]);
    if (r2 = t2, (T(t2) || void 0 !== e2) && !vn(e2))
      return dt(t2) || (t2 = function(e3, t3) {
        if ("function" == typeof r2 && (t3 = r2.call(this, e3, t3)), !vn(t3))
          return t3;
      }), o2[1] = t2, tn.apply(null, o2);
  } });
}
var kn;
en.prototype[Ht] || Z(en.prototype, Ht, en.prototype.valueOf), Jt(en, "Symbol"), te[Mt] = true, Nt("asyncIterator");
var In = W.f;
var Tn = d.Symbol;
if (p && "function" == typeof Tn && (!("description" in Tn.prototype) || void 0 !== Tn().description)) {
  On = {}, En = function() {
    var e2 = arguments.length < 1 || void 0 === arguments[0] ? void 0 : String(arguments[0]), t2 = this instanceof En ? new Tn(e2) : void 0 === e2 ? Tn() : Tn(e2);
    return "" === e2 && (On[t2] = true), t2;
  };
  Le(En, Tn);
  xn = En.prototype = Tn.prototype;
  xn.constructor = En;
  Cn = xn.toString, Rn = "Symbol(test)" == String(Tn("test")), Ln = /^Symbol\((.*)\)[^)]+$/;
  In(xn, "description", { configurable: true, get: function() {
    var e2 = T(this) ? this.valueOf() : this, t2 = Cn.call(e2);
    if (C(On, e2))
      return "";
    var n2 = Rn ? t2.slice(7, -1) : t2.replace(Ln, "$1");
    return "" === n2 ? void 0 : n2;
  } }), Ve({ global: true, forced: true }, { Symbol: En });
}
var On;
var En;
var xn;
var Cn;
var Rn;
var Ln;
Nt("hasInstance"), Nt("isConcatSpreadable"), Nt("iterator"), Nt("match"), Nt("matchAll"), Nt("replace"), Nt("search"), Nt("species"), Nt("split"), Nt("toPrimitive"), Nt("toStringTag"), Nt("unscopables"), Jt(d.JSON, "JSON", true), Jt(Math, "Math", true), Ve({ global: true }, { Reflect: {} }), Jt(d.Reflect, "Reflect", true), de.Symbol;
var Fn;
var jn;
var Un;
var Kn = function(e2) {
  return function(t2, n2) {
    var r2, o2, i2 = String(k(t2)), a2 = me(n2), c2 = i2.length;
    return a2 < 0 || a2 >= c2 ? e2 ? "" : void 0 : (r2 = i2.charCodeAt(a2)) < 55296 || r2 > 56319 || a2 + 1 === c2 || (o2 = i2.charCodeAt(a2 + 1)) < 56320 || o2 > 57343 ? e2 ? i2.charAt(a2) : r2 : e2 ? i2.slice(a2, a2 + 2) : o2 - 56320 + (r2 - 55296 << 10) + 65536;
  };
};
var An = { codeAt: Kn(false), charAt: Kn(true) };
var Pn = !h(function() {
  function e2() {
  }
  return e2.prototype.constructor = null, Object.getPrototypeOf(new e2()) !== e2.prototype;
});
var Wn = ee("IE_PROTO");
var Zn = Object.prototype;
var Vn = Pn ? Object.getPrototypeOf : function(e2) {
  return e2 = E(e2), C(e2, Wn) ? e2[Wn] : "function" == typeof e2.constructor && e2 instanceof e2.constructor ? e2.constructor.prototype : e2 instanceof Object ? Zn : null;
};
var Xn = Qe("iterator");
var Nn = false;
[].keys && ("next" in (Un = [].keys()) ? (jn = Vn(Vn(Un))) !== Object.prototype && (Fn = jn) : Nn = true), (null == Fn || h(function() {
  var e2 = {};
  return Fn[Xn].call(e2) !== e2;
})) && (Fn = {}), C(Fn, Xn) || Z(Fn, Xn, function() {
  return this;
});
var Dn = { IteratorPrototype: Fn, BUGGY_SAFARI_ITERATORS: Nn };
var zn = {};
var Jn = Dn.IteratorPrototype;
var Yn = function() {
  return this;
};
var Gn = Object.setPrototypeOf || ("__proto__" in {} ? function() {
  var e2, t2 = false, n2 = {};
  try {
    (e2 = Object.getOwnPropertyDescriptor(Object.prototype, "__proto__").set).call(n2, []), t2 = n2 instanceof Array;
  } catch (e3) {
  }
  return function(n3, r2) {
    return A(n3), function(e3) {
      if (!T(e3) && null !== e3)
        throw TypeError("Can't set " + String(e3) + " as a prototype");
    }(r2), t2 ? e2.call(n3, r2) : n3.__proto__ = r2, n3;
  };
}() : void 0);
var Bn = Dn.IteratorPrototype;
var Mn = Dn.BUGGY_SAFARI_ITERATORS;
var Hn = Qe("iterator");
var qn = function() {
  return this;
};
var Qn = function(e2, t2, n2, r2, o2, i2, a2) {
  !function(e3, t3, n3) {
    var r3 = t3 + " Iterator";
    e3.prototype = Kt(Jn, { next: g(1, n3) }), Jt(e3, r3, false), zn[r3] = Yn;
  }(n2, t2, r2);
  var c2, s2, u2, l2 = function(e3) {
    if (e3 === o2 && y2)
      return y2;
    if (!Mn && e3 in h2)
      return h2[e3];
    switch (e3) {
      case "keys":
      case "values":
      case "entries":
        return function() {
          return new n2(this, e3);
        };
    }
    return function() {
      return new n2(this);
    };
  }, f2 = t2 + " Iterator", d2 = false, h2 = e2.prototype, p2 = h2[Hn] || h2["@@iterator"] || o2 && h2[o2], y2 = !Mn && p2 || l2(o2), v2 = "Array" == t2 && h2.entries || p2;
  if (v2 && (c2 = Vn(v2.call(new e2())), Bn !== Object.prototype && c2.next && (Vn(c2) !== Bn && (Gn ? Gn(c2, Bn) : "function" != typeof c2[Hn] && Z(c2, Hn, qn)), Jt(c2, f2, true))), "values" == o2 && p2 && "values" !== p2.name && (d2 = true, y2 = function() {
    return p2.call(this);
  }), h2[Hn] !== y2 && Z(h2, Hn, y2), zn[t2] = y2, o2)
    if (s2 = { values: l2("values"), keys: i2 ? y2 : l2("keys"), entries: l2("entries") }, a2)
      for (u2 in s2)
        (Mn || d2 || !(u2 in h2)) && fe(h2, u2, s2[u2]);
    else
      Ve({ target: t2, proto: true, forced: Mn || d2 }, s2);
  return s2;
};
var $n = An.charAt;
var er = le.set;
var tr = le.getterFor("String Iterator");
Qn(String, "String", function(e2) {
  er(this, { type: "String Iterator", string: String(e2), index: 0 });
}, function() {
  var e2, t2 = tr(this), n2 = t2.string, r2 = t2.index;
  return r2 >= n2.length ? { value: void 0, done: true } : (e2 = $n(n2, r2), t2.index += e2.length, { value: e2, done: false });
});
var nr = function(e2) {
  var t2 = e2.return;
  if (void 0 !== t2)
    return A(t2.call(e2)).value;
};
var rr = function(e2, t2, n2, r2) {
  try {
    return r2 ? t2(A(n2)[0], n2[1]) : t2(n2);
  } catch (t3) {
    throw nr(e2), t3;
  }
};
var or = Qe("iterator");
var ir = Array.prototype;
var ar = function(e2) {
  return void 0 !== e2 && (zn.Array === e2 || ir[or] === e2);
};
var cr = Qe("iterator");
var sr = function(e2) {
  if (null != e2)
    return e2[cr] || e2["@@iterator"] || zn[Tt(e2)];
};
var ur = Qe("iterator");
var lr = false;
try {
  fr = 0, dr = { next: function() {
    return { done: !!fr++ };
  }, return: function() {
    lr = true;
  } };
  dr[ur] = function() {
    return this;
  }, Array.from(dr, function() {
    throw 2;
  });
} catch (e2) {
}
var fr;
var dr;
var hr = function(e2, t2) {
  if (!t2 && !lr)
    return false;
  var n2 = false;
  try {
    var r2 = {};
    r2[ur] = function() {
      return { next: function() {
        return { done: n2 = true };
      } };
    }, e2(r2);
  } catch (e3) {
  }
  return n2;
};
var pr = !hr(function(e2) {
  Array.from(e2);
});
Ve({ target: "Array", stat: true, forced: pr }, { from: function(e2) {
  var t2, n2, r2, o2, i2, a2, c2 = E(e2), s2 = "function" == typeof this ? this : Array, u2 = arguments.length, l2 = u2 > 1 ? arguments[1] : void 0, f2 = void 0 !== l2, d2 = sr(c2), h2 = 0;
  if (f2 && (l2 = ut(l2, u2 > 2 ? arguments[2] : void 0, 2)), null == d2 || s2 == Array && ar(d2))
    for (n2 = new s2(t2 = be(c2.length)); t2 > h2; h2++)
      a2 = f2 ? l2(c2[h2], h2) : c2[h2], ht(n2, h2, a2);
  else
    for (i2 = (o2 = d2.call(c2)).next, n2 = new s2(); !(r2 = i2.call(o2)).done; h2++)
      a2 = f2 ? rr(o2, l2, [r2.value, h2], true) : r2.value, ht(n2, h2, a2);
  return n2.length = h2, n2;
} }), de.Array.from;
var yr;
var vr = "undefined" != typeof ArrayBuffer && "undefined" != typeof DataView;
var mr = W.f;
var gr = d.Int8Array;
var br = gr && gr.prototype;
var wr = d.Uint8ClampedArray;
var Sr = wr && wr.prototype;
var _r = gr && Vn(gr);
var kr = br && Vn(br);
var Ir = Object.prototype;
var Tr = Ir.isPrototypeOf;
var Or = Qe("toStringTag");
var Er = Q("TYPED_ARRAY_TAG");
var xr = vr && !!Gn && "Opera" !== Tt(d.opera);
var Cr = { Int8Array: 1, Uint8Array: 1, Uint8ClampedArray: 1, Int16Array: 2, Uint16Array: 2, Int32Array: 4, Uint32Array: 4, Float32Array: 4, Float64Array: 8 };
var Rr = { BigInt64Array: 8, BigUint64Array: 8 };
var Lr = function(e2) {
  if (!T(e2))
    return false;
  var t2 = Tt(e2);
  return C(Cr, t2) || C(Rr, t2);
};
for (yr in Cr)
  d[yr] || (xr = false);
if ((!xr || "function" != typeof _r || _r === Function.prototype) && (_r = function() {
  throw TypeError("Incorrect invocation");
}, xr))
  for (yr in Cr)
    d[yr] && Gn(d[yr], _r);
if ((!xr || !kr || kr === Ir) && (kr = _r.prototype, xr))
  for (yr in Cr)
    d[yr] && Gn(d[yr].prototype, kr);
if (xr && Vn(Sr) !== kr && Gn(Sr, kr), p && !C(kr, Or))
  for (yr in true, mr(kr, Or, { get: function() {
    return T(this) ? this[Er] : void 0;
  } }), Cr)
    d[yr] && Z(d[yr], Er, yr);
var Fr = function(e2) {
  if (Lr(e2))
    return e2;
  throw TypeError("Target is not a typed array");
};
var jr = function(e2) {
  if (Gn) {
    if (Tr.call(_r, e2))
      return e2;
  } else
    for (var t2 in Cr)
      if (C(Cr, yr)) {
        var n2 = d[t2];
        if (n2 && (e2 === n2 || Tr.call(n2, e2)))
          return e2;
      }
  throw TypeError("Target is not a typed array constructor");
};
var Ur = function(e2, t2, n2) {
  if (p) {
    if (n2)
      for (var r2 in Cr) {
        var o2 = d[r2];
        if (o2 && C(o2.prototype, e2))
          try {
            delete o2.prototype[e2];
          } catch (e3) {
          }
      }
    kr[e2] && !n2 || fe(kr, e2, n2 ? t2 : xr && br[e2] || t2);
  }
};
var Kr = Qe("species");
var Ar = Fr;
var Pr = jr;
var Wr = [].slice;
Ur("slice", function(e2, t2) {
  for (var n2 = Wr.call(Ar(this), e2, t2), r2 = function(e3, t3) {
    var n3, r3 = A(e3).constructor;
    return void 0 === r3 || null == (n3 = A(r3)[Kr]) ? t3 : st(n3);
  }(this, this.constructor), o2 = 0, i2 = n2.length, a2 = new (Pr(r2))(i2); i2 > o2; )
    a2[o2] = n2[o2++];
  return a2;
}, h(function() {
  new Int8Array(1).slice();
}));
var Zr = Qe("unscopables");
var Vr = Array.prototype;
null == Vr[Zr] && W.f(Vr, Zr, { configurable: true, value: Kt(null) });
var Xr = function(e2) {
  Vr[Zr][e2] = true;
};
var Nr = ke.includes;
Ve({ target: "Array", proto: true }, { includes: function(e2) {
  return Nr(this, e2, arguments.length > 1 ? arguments[1] : void 0);
} }), Xr("includes"), ft("Array", "includes"), Ve({ target: "String", proto: true, forced: !nt("includes") }, { includes: function(e2) {
  return !!~String(k(this)).indexOf(et(e2), arguments.length > 1 ? arguments[1] : void 0);
} }), ft("String", "includes");
var Dr = !h(function() {
  return Object.isExtensible(Object.preventExtensions({}));
});
var zr = l(function(e2) {
  var t2 = W.f, n2 = Q("meta"), r2 = 0, o2 = Object.isExtensible || function() {
    return true;
  }, i2 = function(e3) {
    t2(e3, n2, { value: { objectID: "O" + r2++, weakData: {} } });
  }, a2 = e2.exports = { REQUIRED: false, fastKey: function(e3, t3) {
    if (!T(e3))
      return "symbol" == typeof e3 ? e3 : ("string" == typeof e3 ? "S" : "P") + e3;
    if (!C(e3, n2)) {
      if (!o2(e3))
        return "F";
      if (!t3)
        return "E";
      i2(e3);
    }
    return e3[n2].objectID;
  }, getWeakData: function(e3, t3) {
    if (!C(e3, n2)) {
      if (!o2(e3))
        return true;
      if (!t3)
        return false;
      i2(e3);
    }
    return e3[n2].weakData;
  }, onFreeze: function(e3) {
    return Dr && a2.REQUIRED && o2(e3) && !C(e3, n2) && i2(e3), e3;
  } };
  te[n2] = true;
});
zr.REQUIRED, zr.fastKey, zr.getWeakData, zr.onFreeze;
var Jr = function(e2, t2) {
  this.stopped = e2, this.result = t2;
};
var Yr = function(e2, t2, n2) {
  var r2, o2, i2, a2, c2, s2, u2, l2 = n2 && n2.that, f2 = !(!n2 || !n2.AS_ENTRIES), d2 = !(!n2 || !n2.IS_ITERATOR), h2 = !(!n2 || !n2.INTERRUPTED), p2 = ut(t2, l2, 1 + f2 + h2), y2 = function(e3) {
    return r2 && nr(r2), new Jr(true, e3);
  }, v2 = function(e3) {
    return f2 ? (A(e3), h2 ? p2(e3[0], e3[1], y2) : p2(e3[0], e3[1])) : h2 ? p2(e3, y2) : p2(e3);
  };
  if (d2)
    r2 = e2;
  else {
    if ("function" != typeof (o2 = sr(e2)))
      throw TypeError("Target is not iterable");
    if (ar(o2)) {
      for (i2 = 0, a2 = be(e2.length); a2 > i2; i2++)
        if ((c2 = v2(e2[i2])) && c2 instanceof Jr)
          return c2;
      return new Jr(false);
    }
    r2 = o2.call(e2);
  }
  for (s2 = r2.next; !(u2 = s2.call(r2)).done; ) {
    try {
      c2 = v2(u2.value);
    } catch (e3) {
      throw nr(r2), e3;
    }
    if ("object" == typeof c2 && c2 && c2 instanceof Jr)
      return c2;
  }
  return new Jr(false);
};
var Gr = function(e2, t2, n2) {
  if (!(e2 instanceof t2))
    throw TypeError("Incorrect " + (n2 ? n2 + " " : "") + "invocation");
  return e2;
};
var Br = function(e2, t2, n2) {
  for (var r2 in t2)
    fe(e2, r2, t2[r2], n2);
  return e2;
};
var Mr = Qe("species");
var Hr = W.f;
var qr = zr.fastKey;
var Qr = le.set;
var $r = le.getterFor;
var eo = { getConstructor: function(e2, t2, n2, r2) {
  var o2 = e2(function(e3, i3) {
    Gr(e3, o2, t2), Qr(e3, { type: t2, index: Kt(null), first: void 0, last: void 0, size: 0 }), p || (e3.size = 0), null != i3 && Yr(i3, e3[r2], { that: e3, AS_ENTRIES: n2 });
  }), i2 = $r(t2), a2 = function(e3, t3, n3) {
    var r3, o3, a3 = i2(e3), s2 = c2(e3, t3);
    return s2 ? s2.value = n3 : (a3.last = s2 = { index: o3 = qr(t3, true), key: t3, value: n3, previous: r3 = a3.last, next: void 0, removed: false }, a3.first || (a3.first = s2), r3 && (r3.next = s2), p ? a3.size++ : e3.size++, "F" !== o3 && (a3.index[o3] = s2)), e3;
  }, c2 = function(e3, t3) {
    var n3, r3 = i2(e3), o3 = qr(t3);
    if ("F" !== o3)
      return r3.index[o3];
    for (n3 = r3.first; n3; n3 = n3.next)
      if (n3.key == t3)
        return n3;
  };
  return Br(o2.prototype, { clear: function() {
    for (var e3 = i2(this), t3 = e3.index, n3 = e3.first; n3; )
      n3.removed = true, n3.previous && (n3.previous = n3.previous.next = void 0), delete t3[n3.index], n3 = n3.next;
    e3.first = e3.last = void 0, p ? e3.size = 0 : this.size = 0;
  }, delete: function(e3) {
    var t3 = this, n3 = i2(t3), r3 = c2(t3, e3);
    if (r3) {
      var o3 = r3.next, a3 = r3.previous;
      delete n3.index[r3.index], r3.removed = true, a3 && (a3.next = o3), o3 && (o3.previous = a3), n3.first == r3 && (n3.first = o3), n3.last == r3 && (n3.last = a3), p ? n3.size-- : t3.size--;
    }
    return !!r3;
  }, forEach: function(e3) {
    for (var t3, n3 = i2(this), r3 = ut(e3, arguments.length > 1 ? arguments[1] : void 0, 3); t3 = t3 ? t3.next : n3.first; )
      for (r3(t3.value, t3.key, this); t3 && t3.removed; )
        t3 = t3.previous;
  }, has: function(e3) {
    return !!c2(this, e3);
  } }), Br(o2.prototype, n2 ? { get: function(e3) {
    var t3 = c2(this, e3);
    return t3 && t3.value;
  }, set: function(e3, t3) {
    return a2(this, 0 === e3 ? 0 : e3, t3);
  } } : { add: function(e3) {
    return a2(this, e3 = 0 === e3 ? 0 : e3, e3);
  } }), p && Hr(o2.prototype, "size", { get: function() {
    return i2(this).size;
  } }), o2;
}, setStrong: function(e2, t2, n2) {
  var r2 = t2 + " Iterator", o2 = $r(t2), i2 = $r(r2);
  Qn(e2, t2, function(e3, t3) {
    Qr(this, { type: r2, target: e3, state: o2(e3), kind: t3, last: void 0 });
  }, function() {
    for (var e3 = i2(this), t3 = e3.kind, n3 = e3.last; n3 && n3.removed; )
      n3 = n3.previous;
    return e3.target && (e3.last = n3 = n3 ? n3.next : e3.state.first) ? "keys" == t3 ? { value: n3.key, done: false } : "values" == t3 ? { value: n3.value, done: false } : { value: [n3.key, n3.value], done: false } : (e3.target = void 0, { value: void 0, done: true });
  }, n2 ? "entries" : "values", !n2, true), function(e3) {
    var t3 = pe(e3), n3 = W.f;
    p && t3 && !t3[Mr] && n3(t3, Mr, { configurable: true, get: function() {
      return this;
    } });
  }(t2);
} };
!function(e2, t2, n2) {
  var r2 = -1 !== e2.indexOf("Map"), o2 = -1 !== e2.indexOf("Weak"), i2 = r2 ? "set" : "add", a2 = d[e2], c2 = a2 && a2.prototype, s2 = a2, u2 = {}, l2 = function(e3) {
    var t3 = c2[e3];
    fe(c2, e3, "add" == e3 ? function(e4) {
      return t3.call(this, 0 === e4 ? 0 : e4), this;
    } : "delete" == e3 ? function(e4) {
      return !(o2 && !T(e4)) && t3.call(this, 0 === e4 ? 0 : e4);
    } : "get" == e3 ? function(e4) {
      return o2 && !T(e4) ? void 0 : t3.call(this, 0 === e4 ? 0 : e4);
    } : "has" == e3 ? function(e4) {
      return !(o2 && !T(e4)) && t3.call(this, 0 === e4 ? 0 : e4);
    } : function(e4, n3) {
      return t3.call(this, 0 === e4 ? 0 : e4, n3), this;
    });
  };
  if (We(e2, "function" != typeof a2 || !(o2 || c2.forEach && !h(function() {
    new a2().entries().next();
  }))))
    s2 = n2.getConstructor(t2, e2, r2, i2), zr.REQUIRED = true;
  else if (We(e2, true)) {
    var f2 = new s2(), p2 = f2[i2](o2 ? {} : -0, 1) != f2, y2 = h(function() {
      f2.has(1);
    }), v2 = hr(function(e3) {
      new a2(e3);
    }), m2 = !o2 && h(function() {
      for (var e3 = new a2(), t3 = 5; t3--; )
        e3[i2](t3, t3);
      return !e3.has(-0);
    });
    v2 || ((s2 = t2(function(t3, n3) {
      Gr(t3, s2, e2);
      var o3 = function(e3, t4, n4) {
        var r3, o4;
        return Gn && "function" == typeof (r3 = t4.constructor) && r3 !== n4 && T(o4 = r3.prototype) && o4 !== n4.prototype && Gn(e3, o4), e3;
      }(new a2(), t3, s2);
      return null != n3 && Yr(n3, o3[i2], { that: o3, AS_ENTRIES: r2 }), o3;
    })).prototype = c2, c2.constructor = s2), (y2 || m2) && (l2("delete"), l2("has"), r2 && l2("get")), (m2 || p2) && l2(i2), o2 && c2.clear && delete c2.clear;
  }
  u2[e2] = s2, Ve({ global: true, forced: s2 != a2 }, u2), Jt(s2, e2), o2 || n2.setStrong(s2, e2, r2);
}("Set", function(e2) {
  return function() {
    return e2(this, arguments.length ? arguments[0] : void 0);
  };
}, eo);
var to = { CSSRuleList: 0, CSSStyleDeclaration: 0, CSSValueList: 0, ClientRectList: 0, DOMRectList: 0, DOMStringList: 0, DOMTokenList: 1, DataTransferItemList: 0, FileList: 0, HTMLAllCollection: 0, HTMLCollection: 0, HTMLFormElement: 0, HTMLSelectElement: 0, MediaList: 0, MimeTypeArray: 0, NamedNodeMap: 0, NodeList: 1, PaintRequestList: 0, Plugin: 0, PluginArray: 0, SVGLengthList: 0, SVGNumberList: 0, SVGPathSegList: 0, SVGPointList: 0, SVGStringList: 0, SVGTransformList: 0, SourceBufferList: 0, StyleSheetList: 0, TextTrackCueList: 0, TextTrackList: 0, TouchList: 0 };
var no = le.set;
var ro = le.getterFor("Array Iterator");
var oo = Qn(Array, "Array", function(e2, t2) {
  no(this, { type: "Array Iterator", target: I(e2), index: 0, kind: t2 });
}, function() {
  var e2 = ro(this), t2 = e2.target, n2 = e2.kind, r2 = e2.index++;
  return !t2 || r2 >= t2.length ? (e2.target = void 0, { value: void 0, done: true }) : "keys" == n2 ? { value: r2, done: false } : "values" == n2 ? { value: t2[r2], done: false } : { value: [r2, t2[r2]], done: false };
}, "values");
zn.Arguments = zn.Array, Xr("keys"), Xr("values"), Xr("entries");
var io = Qe("iterator");
var ao = Qe("toStringTag");
var co = oo.values;
for (so in to) {
  uo = d[so], lo = uo && uo.prototype;
  if (lo) {
    if (lo[io] !== co)
      try {
        Z(lo, io, co);
      } catch (e2) {
        lo[io] = co;
      }
    if (lo[ao] || Z(lo, ao, so), to[so]) {
      for (fo in oo)
        if (lo[fo] !== oo[fo])
          try {
            Z(lo, fo, oo[fo]);
          } catch (e2) {
            lo[fo] = oo[fo];
          }
    }
  }
}
var uo;
var lo;
var fo;
var so;
function ho(e2) {
  var t2 = this.constructor;
  return this.then(function(n2) {
    return t2.resolve(e2()).then(function() {
      return n2;
    });
  }, function(n2) {
    return t2.resolve(e2()).then(function() {
      return t2.reject(n2);
    });
  });
}
function po(e2) {
  return new this(function(t2, n2) {
    if (!e2 || void 0 === e2.length)
      return n2(new TypeError(typeof e2 + " " + e2 + " is not iterable(cannot read property Symbol(Symbol.iterator))"));
    var r2 = Array.prototype.slice.call(e2);
    if (0 === r2.length)
      return t2([]);
    var o2 = r2.length;
    function i2(e3, n3) {
      if (n3 && ("object" == typeof n3 || "function" == typeof n3)) {
        var a3 = n3.then;
        if ("function" == typeof a3)
          return void a3.call(n3, function(t3) {
            i2(e3, t3);
          }, function(n4) {
            r2[e3] = { status: "rejected", reason: n4 }, 0 == --o2 && t2(r2);
          });
      }
      r2[e3] = { status: "fulfilled", value: n3 }, 0 == --o2 && t2(r2);
    }
    for (var a2 = 0; a2 < r2.length; a2++)
      i2(a2, r2[a2]);
  });
}
de.Set;
var yo = setTimeout;
function vo(e2) {
  return Boolean(e2 && void 0 !== e2.length);
}
function mo() {
}
function go(e2) {
  if (!(this instanceof go))
    throw new TypeError("Promises must be constructed via new");
  if ("function" != typeof e2)
    throw new TypeError("not a function");
  this._state = 0, this._handled = false, this._value = void 0, this._deferreds = [], Io(e2, this);
}
function bo(e2, t2) {
  for (; 3 === e2._state; )
    e2 = e2._value;
  0 !== e2._state ? (e2._handled = true, go._immediateFn(function() {
    var n2 = 1 === e2._state ? t2.onFulfilled : t2.onRejected;
    if (null !== n2) {
      var r2;
      try {
        r2 = n2(e2._value);
      } catch (e3) {
        return void So(t2.promise, e3);
      }
      wo(t2.promise, r2);
    } else
      (1 === e2._state ? wo : So)(t2.promise, e2._value);
  })) : e2._deferreds.push(t2);
}
function wo(e2, t2) {
  try {
    if (t2 === e2)
      throw new TypeError("A promise cannot be resolved with itself.");
    if (t2 && ("object" == typeof t2 || "function" == typeof t2)) {
      var n2 = t2.then;
      if (t2 instanceof go)
        return e2._state = 3, e2._value = t2, void _o(e2);
      if ("function" == typeof n2)
        return void Io((r2 = n2, o2 = t2, function() {
          r2.apply(o2, arguments);
        }), e2);
    }
    e2._state = 1, e2._value = t2, _o(e2);
  } catch (t3) {
    So(e2, t3);
  }
  var r2, o2;
}
function So(e2, t2) {
  e2._state = 2, e2._value = t2, _o(e2);
}
function _o(e2) {
  2 === e2._state && 0 === e2._deferreds.length && go._immediateFn(function() {
    e2._handled || go._unhandledRejectionFn(e2._value);
  });
  for (var t2 = 0, n2 = e2._deferreds.length; t2 < n2; t2++)
    bo(e2, e2._deferreds[t2]);
  e2._deferreds = null;
}
function ko(e2, t2, n2) {
  this.onFulfilled = "function" == typeof e2 ? e2 : null, this.onRejected = "function" == typeof t2 ? t2 : null, this.promise = n2;
}
function Io(e2, t2) {
  var n2 = false;
  try {
    e2(function(e3) {
      n2 || (n2 = true, wo(t2, e3));
    }, function(e3) {
      n2 || (n2 = true, So(t2, e3));
    });
  } catch (e3) {
    if (n2)
      return;
    n2 = true, So(t2, e3);
  }
}
go.prototype.catch = function(e2) {
  return this.then(null, e2);
}, go.prototype.then = function(e2, t2) {
  var n2 = new this.constructor(mo);
  return bo(this, new ko(e2, t2, n2)), n2;
}, go.prototype.finally = ho, go.all = function(e2) {
  return new go(function(t2, n2) {
    if (!vo(e2))
      return n2(new TypeError("Promise.all accepts an array"));
    var r2 = Array.prototype.slice.call(e2);
    if (0 === r2.length)
      return t2([]);
    var o2 = r2.length;
    function i2(e3, a3) {
      try {
        if (a3 && ("object" == typeof a3 || "function" == typeof a3)) {
          var c2 = a3.then;
          if ("function" == typeof c2)
            return void c2.call(a3, function(t3) {
              i2(e3, t3);
            }, n2);
        }
        r2[e3] = a3, 0 == --o2 && t2(r2);
      } catch (e4) {
        n2(e4);
      }
    }
    for (var a2 = 0; a2 < r2.length; a2++)
      i2(a2, r2[a2]);
  });
}, go.allSettled = po, go.resolve = function(e2) {
  return e2 && "object" == typeof e2 && e2.constructor === go ? e2 : new go(function(t2) {
    t2(e2);
  });
}, go.reject = function(e2) {
  return new go(function(t2, n2) {
    n2(e2);
  });
}, go.race = function(e2) {
  return new go(function(t2, n2) {
    if (!vo(e2))
      return n2(new TypeError("Promise.race accepts an array"));
    for (var r2 = 0, o2 = e2.length; r2 < o2; r2++)
      go.resolve(e2[r2]).then(t2, n2);
  });
}, go._immediateFn = "function" == typeof setImmediate && function(e2) {
  setImmediate(e2);
} || function(e2) {
  yo(e2, 0);
}, go._unhandledRejectionFn = function(e2) {
  "undefined" != typeof console && console && console.warn("Possible Unhandled Promise Rejection:", e2);
};
var To = function() {
  if ("undefined" != typeof self)
    return self;
  if ("undefined" != typeof window)
    return window;
  if ("undefined" != typeof global)
    return global;
  throw new Error("unable to locate global object");
}();
"function" != typeof To.Promise ? To.Promise = go : To.Promise.prototype.finally ? To.Promise.allSettled || (To.Promise.allSettled = po) : To.Promise.prototype.finally = ho, function(e2) {
  function t2() {
  }
  function n2(e3, t3) {
    if (e3 = void 0 === e3 ? "utf-8" : e3, t3 = void 0 === t3 ? { fatal: false } : t3, -1 === o2.indexOf(e3.toLowerCase()))
      throw new RangeError("Failed to construct 'TextDecoder': The encoding label provided ('" + e3 + "') is invalid.");
    if (t3.fatal)
      throw Error("Failed to construct 'TextDecoder': the 'fatal' option is unsupported.");
  }
  function r2(e3) {
    for (var t3 = 0, n3 = Math.min(65536, e3.length + 1), r3 = new Uint16Array(n3), o3 = [], i3 = 0; ; ) {
      var a2 = t3 < e3.length;
      if (!a2 || i3 >= n3 - 1) {
        if (o3.push(String.fromCharCode.apply(null, r3.subarray(0, i3))), !a2)
          return o3.join("");
        e3 = e3.subarray(t3), i3 = t3 = 0;
      }
      if (0 == (128 & (a2 = e3[t3++])))
        r3[i3++] = a2;
      else if (192 == (224 & a2)) {
        var c2 = 63 & e3[t3++];
        r3[i3++] = (31 & a2) << 6 | c2;
      } else if (224 == (240 & a2)) {
        c2 = 63 & e3[t3++];
        var s2 = 63 & e3[t3++];
        r3[i3++] = (31 & a2) << 12 | c2 << 6 | s2;
      } else if (240 == (248 & a2)) {
        65535 < (a2 = (7 & a2) << 18 | (c2 = 63 & e3[t3++]) << 12 | (s2 = 63 & e3[t3++]) << 6 | 63 & e3[t3++]) && (a2 -= 65536, r3[i3++] = a2 >>> 10 & 1023 | 55296, a2 = 56320 | 1023 & a2), r3[i3++] = a2;
      }
    }
  }
  if (e2.TextEncoder && e2.TextDecoder)
    return false;
  var o2 = ["utf-8", "utf8", "unicode-1-1-utf-8"];
  Object.defineProperty(t2.prototype, "encoding", { value: "utf-8" }), t2.prototype.encode = function(e3, t3) {
    if ((t3 = void 0 === t3 ? { stream: false } : t3).stream)
      throw Error("Failed to encode: the 'stream' option is unsupported.");
    t3 = 0;
    for (var n3 = e3.length, r3 = 0, o3 = Math.max(32, n3 + (n3 >>> 1) + 7), i3 = new Uint8Array(o3 >>> 3 << 3); t3 < n3; ) {
      var a2 = e3.charCodeAt(t3++);
      if (55296 <= a2 && 56319 >= a2) {
        if (t3 < n3) {
          var c2 = e3.charCodeAt(t3);
          56320 == (64512 & c2) && (++t3, a2 = ((1023 & a2) << 10) + (1023 & c2) + 65536);
        }
        if (55296 <= a2 && 56319 >= a2)
          continue;
      }
      if (r3 + 4 > i3.length && (o3 += 8, o3 = (o3 *= 1 + t3 / e3.length * 2) >>> 3 << 3, (c2 = new Uint8Array(o3)).set(i3), i3 = c2), 0 == (4294967168 & a2))
        i3[r3++] = a2;
      else {
        if (0 == (4294965248 & a2))
          i3[r3++] = a2 >>> 6 & 31 | 192;
        else if (0 == (4294901760 & a2))
          i3[r3++] = a2 >>> 12 & 15 | 224, i3[r3++] = a2 >>> 6 & 63 | 128;
        else {
          if (0 != (4292870144 & a2))
            continue;
          i3[r3++] = a2 >>> 18 & 7 | 240, i3[r3++] = a2 >>> 12 & 63 | 128, i3[r3++] = a2 >>> 6 & 63 | 128;
        }
        i3[r3++] = 63 & a2 | 128;
      }
    }
    return i3.slice ? i3.slice(0, r3) : i3.subarray(0, r3);
  }, Object.defineProperty(n2.prototype, "encoding", { value: "utf-8" }), Object.defineProperty(n2.prototype, "fatal", { value: false }), Object.defineProperty(n2.prototype, "ignoreBOM", { value: false });
  var i2 = r2;
  "function" == typeof Buffer && Buffer.from ? i2 = function(e3) {
    return Buffer.from(e3.buffer, e3.byteOffset, e3.byteLength).toString("utf-8");
  } : "function" == typeof Blob && "function" == typeof URL && "function" == typeof URL.createObjectURL && (i2 = function(e3) {
    var t3 = URL.createObjectURL(new Blob([e3], { type: "text/plain;charset=UTF-8" }));
    try {
      var n3 = new XMLHttpRequest();
      return n3.open("GET", t3, false), n3.send(), n3.responseText;
    } catch (t4) {
      return r2(e3);
    } finally {
      URL.revokeObjectURL(t3);
    }
  }), n2.prototype.decode = function(e3, t3) {
    if ((t3 = void 0 === t3 ? { stream: false } : t3).stream)
      throw Error("Failed to decode: the 'stream' option is unsupported.");
    return e3 = e3 instanceof Uint8Array ? e3 : e3.buffer instanceof ArrayBuffer ? new Uint8Array(e3.buffer) : new Uint8Array(e3), i2(e3);
  }, e2.TextEncoder = t2, e2.TextDecoder = n2;
}("undefined" != typeof window ? window : s), function() {
  function e2(e3, t3) {
    if (!(e3 instanceof t3))
      throw new TypeError("Cannot call a class as a function");
  }
  function t2(e3, t3) {
    for (var n3 = 0; n3 < t3.length; n3++) {
      var r3 = t3[n3];
      r3.enumerable = r3.enumerable || false, r3.configurable = true, "value" in r3 && (r3.writable = true), Object.defineProperty(e3, r3.key, r3);
    }
  }
  function n2(e3, n3, r3) {
    return n3 && t2(e3.prototype, n3), r3 && t2(e3, r3), e3;
  }
  function r2(e3, t3) {
    if ("function" != typeof t3 && null !== t3)
      throw new TypeError("Super expression must either be null or a function");
    e3.prototype = Object.create(t3 && t3.prototype, { constructor: { value: e3, writable: true, configurable: true } }), t3 && i2(e3, t3);
  }
  function o2(e3) {
    return (o2 = Object.setPrototypeOf ? Object.getPrototypeOf : function(e4) {
      return e4.__proto__ || Object.getPrototypeOf(e4);
    })(e3);
  }
  function i2(e3, t3) {
    return (i2 = Object.setPrototypeOf || function(e4, t4) {
      return e4.__proto__ = t4, e4;
    })(e3, t3);
  }
  function a2() {
    if ("undefined" == typeof Reflect || !Reflect.construct)
      return false;
    if (Reflect.construct.sham)
      return false;
    if ("function" == typeof Proxy)
      return true;
    try {
      return Boolean.prototype.valueOf.call(Reflect.construct(Boolean, [], function() {
      })), true;
    } catch (e3) {
      return false;
    }
  }
  function c2(e3) {
    if (void 0 === e3)
      throw new ReferenceError("this hasn't been initialised - super() hasn't been called");
    return e3;
  }
  function u2(e3, t3) {
    return !t3 || "object" != typeof t3 && "function" != typeof t3 ? c2(e3) : t3;
  }
  function l2(e3) {
    var t3 = a2();
    return function() {
      var n3, r3 = o2(e3);
      if (t3) {
        var i3 = o2(this).constructor;
        n3 = Reflect.construct(r3, arguments, i3);
      } else
        n3 = r3.apply(this, arguments);
      return u2(this, n3);
    };
  }
  function f2(e3, t3) {
    for (; !Object.prototype.hasOwnProperty.call(e3, t3) && null !== (e3 = o2(e3)); )
      ;
    return e3;
  }
  function d2(e3, t3, n3) {
    return (d2 = "undefined" != typeof Reflect && Reflect.get ? Reflect.get : function(e4, t4, n4) {
      var r3 = f2(e4, t4);
      if (r3) {
        var o3 = Object.getOwnPropertyDescriptor(r3, t4);
        return o3.get ? o3.get.call(n4) : o3.value;
      }
    })(e3, t3, n3 || e3);
  }
  var h2 = function() {
    function t3() {
      e2(this, t3), Object.defineProperty(this, "listeners", { value: {}, writable: true, configurable: true });
    }
    return n2(t3, [{ key: "addEventListener", value: function(e3, t4, n3) {
      e3 in this.listeners || (this.listeners[e3] = []), this.listeners[e3].push({ callback: t4, options: n3 });
    } }, { key: "removeEventListener", value: function(e3, t4) {
      if (e3 in this.listeners) {
        for (var n3 = this.listeners[e3], r3 = 0, o3 = n3.length; r3 < o3; r3++)
          if (n3[r3].callback === t4)
            return void n3.splice(r3, 1);
      }
    } }, { key: "dispatchEvent", value: function(e3) {
      if (e3.type in this.listeners) {
        for (var t4 = this.listeners[e3.type].slice(), n3 = 0, r3 = t4.length; n3 < r3; n3++) {
          var o3 = t4[n3];
          try {
            o3.callback.call(this, e3);
          } catch (e4) {
            Promise.resolve().then(function() {
              throw e4;
            });
          }
          o3.options && o3.options.once && this.removeEventListener(e3.type, o3.callback);
        }
        return !e3.defaultPrevented;
      }
    } }]), t3;
  }(), p2 = function(t3) {
    r2(a3, t3);
    var i3 = l2(a3);
    function a3() {
      var t4;
      return e2(this, a3), (t4 = i3.call(this)).listeners || h2.call(c2(t4)), Object.defineProperty(c2(t4), "aborted", { value: false, writable: true, configurable: true }), Object.defineProperty(c2(t4), "onabort", { value: null, writable: true, configurable: true }), t4;
    }
    return n2(a3, [{ key: "toString", value: function() {
      return "[object AbortSignal]";
    } }, { key: "dispatchEvent", value: function(e3) {
      "abort" === e3.type && (this.aborted = true, "function" == typeof this.onabort && this.onabort.call(this, e3)), d2(o2(a3.prototype), "dispatchEvent", this).call(this, e3);
    } }]), a3;
  }(h2), y2 = function() {
    function t3() {
      e2(this, t3), Object.defineProperty(this, "signal", { value: new p2(), writable: true, configurable: true });
    }
    return n2(t3, [{ key: "abort", value: function() {
      var e3;
      try {
        e3 = new Event("abort");
      } catch (t4) {
        "undefined" != typeof document ? document.createEvent ? (e3 = document.createEvent("Event")).initEvent("abort", false, false) : (e3 = document.createEventObject()).type = "abort" : e3 = { type: "abort", bubbles: false, cancelable: false };
      }
      this.signal.dispatchEvent(e3);
    } }, { key: "toString", value: function() {
      return "[object AbortController]";
    } }]), t3;
  }();
  function v2(e3) {
    return e3.__FORCE_INSTALL_ABORTCONTROLLER_POLYFILL ? (console.log("__FORCE_INSTALL_ABORTCONTROLLER_POLYFILL=true is set, will force install polyfill"), true) : "function" == typeof e3.Request && !e3.Request.prototype.hasOwnProperty("signal") || !e3.AbortController;
  }
  "undefined" != typeof Symbol && Symbol.toStringTag && (y2.prototype[Symbol.toStringTag] = "AbortController", p2.prototype[Symbol.toStringTag] = "AbortSignal"), function(e3) {
    v2(e3) && (e3.AbortController = y2, e3.AbortSignal = p2);
  }("undefined" != typeof self ? self : s);
}();
var Oo = l(function(e2, t2) {
  Object.defineProperty(t2, "__esModule", { value: true });
  var n2 = function() {
    function e3() {
      var e4 = this;
      this.locked = /* @__PURE__ */ new Map(), this.addToLocked = function(t3, n3) {
        var r2 = e4.locked.get(t3);
        void 0 === r2 ? void 0 === n3 ? e4.locked.set(t3, []) : e4.locked.set(t3, [n3]) : void 0 !== n3 && (r2.unshift(n3), e4.locked.set(t3, r2));
      }, this.isLocked = function(t3) {
        return e4.locked.has(t3);
      }, this.lock = function(t3) {
        return new Promise(function(n3, r2) {
          e4.isLocked(t3) ? e4.addToLocked(t3, n3) : (e4.addToLocked(t3), n3());
        });
      }, this.unlock = function(t3) {
        var n3 = e4.locked.get(t3);
        if (void 0 !== n3 && 0 !== n3.length) {
          var r2 = n3.pop();
          e4.locked.set(t3, n3), void 0 !== r2 && setTimeout(r2, 0);
        } else
          e4.locked.delete(t3);
      };
    }
    return e3.getInstance = function() {
      return void 0 === e3.instance && (e3.instance = new e3()), e3.instance;
    }, e3;
  }();
  t2.default = function() {
    return n2.getInstance();
  };
});
u(Oo);
var Eo = u(l(function(e2, t2) {
  var n2 = s && s.__awaiter || function(e3, t3, n3, r3) {
    return new (n3 || (n3 = Promise))(function(o3, i3) {
      function a3(e4) {
        try {
          s2(r3.next(e4));
        } catch (e5) {
          i3(e5);
        }
      }
      function c3(e4) {
        try {
          s2(r3.throw(e4));
        } catch (e5) {
          i3(e5);
        }
      }
      function s2(e4) {
        e4.done ? o3(e4.value) : new n3(function(t4) {
          t4(e4.value);
        }).then(a3, c3);
      }
      s2((r3 = r3.apply(e3, t3 || [])).next());
    });
  }, r2 = s && s.__generator || function(e3, t3) {
    var n3, r3, o3, i3, a3 = { label: 0, sent: function() {
      if (1 & o3[0])
        throw o3[1];
      return o3[1];
    }, trys: [], ops: [] };
    return i3 = { next: c3(0), throw: c3(1), return: c3(2) }, "function" == typeof Symbol && (i3[Symbol.iterator] = function() {
      return this;
    }), i3;
    function c3(i4) {
      return function(c4) {
        return function(i5) {
          if (n3)
            throw new TypeError("Generator is already executing.");
          for (; a3; )
            try {
              if (n3 = 1, r3 && (o3 = 2 & i5[0] ? r3.return : i5[0] ? r3.throw || ((o3 = r3.return) && o3.call(r3), 0) : r3.next) && !(o3 = o3.call(r3, i5[1])).done)
                return o3;
              switch (r3 = 0, o3 && (i5 = [2 & i5[0], o3.value]), i5[0]) {
                case 0:
                case 1:
                  o3 = i5;
                  break;
                case 4:
                  return a3.label++, { value: i5[1], done: false };
                case 5:
                  a3.label++, r3 = i5[1], i5 = [0];
                  continue;
                case 7:
                  i5 = a3.ops.pop(), a3.trys.pop();
                  continue;
                default:
                  if (!(o3 = a3.trys, (o3 = o3.length > 0 && o3[o3.length - 1]) || 6 !== i5[0] && 2 !== i5[0])) {
                    a3 = 0;
                    continue;
                  }
                  if (3 === i5[0] && (!o3 || i5[1] > o3[0] && i5[1] < o3[3])) {
                    a3.label = i5[1];
                    break;
                  }
                  if (6 === i5[0] && a3.label < o3[1]) {
                    a3.label = o3[1], o3 = i5;
                    break;
                  }
                  if (o3 && a3.label < o3[2]) {
                    a3.label = o3[2], a3.ops.push(i5);
                    break;
                  }
                  o3[2] && a3.ops.pop(), a3.trys.pop();
                  continue;
              }
              i5 = t3.call(e3, a3);
            } catch (e4) {
              i5 = [6, e4], r3 = 0;
            } finally {
              n3 = o3 = 0;
            }
          if (5 & i5[0])
            throw i5[1];
          return { value: i5[0] ? i5[1] : void 0, done: true };
        }([i4, c4]);
      };
    }
  };
  Object.defineProperty(t2, "__esModule", { value: true });
  var o2 = "browser-tabs-lock-key";
  function i2(e3) {
    return new Promise(function(t3) {
      return setTimeout(t3, e3);
    });
  }
  function a2(e3) {
    for (var t3 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXTZabcdefghiklmnopqrstuvwxyz", n3 = "", r3 = 0; r3 < e3; r3++) {
      n3 += t3[Math.floor(Math.random() * t3.length)];
    }
    return n3;
  }
  var c2 = function() {
    function e3() {
      this.acquiredIatSet = /* @__PURE__ */ new Set(), this.id = Date.now().toString() + a2(15), this.acquireLock = this.acquireLock.bind(this), this.releaseLock = this.releaseLock.bind(this), this.releaseLock__private__ = this.releaseLock__private__.bind(this), this.waitForSomethingToChange = this.waitForSomethingToChange.bind(this), this.refreshLockWhileAcquired = this.refreshLockWhileAcquired.bind(this), void 0 === e3.waiters && (e3.waiters = []);
    }
    return e3.prototype.acquireLock = function(t3, c3) {
      return void 0 === c3 && (c3 = 5e3), n2(this, void 0, void 0, function() {
        var n3, s2, u2, l2, f2, d2;
        return r2(this, function(r3) {
          switch (r3.label) {
            case 0:
              n3 = Date.now() + a2(4), s2 = Date.now() + c3, u2 = o2 + "-" + t3, l2 = window.localStorage, r3.label = 1;
            case 1:
              return Date.now() < s2 ? [4, i2(30)] : [3, 8];
            case 2:
              return r3.sent(), null !== l2.getItem(u2) ? [3, 5] : (f2 = this.id + "-" + t3 + "-" + n3, [4, i2(Math.floor(25 * Math.random()))]);
            case 3:
              return r3.sent(), l2.setItem(u2, JSON.stringify({ id: this.id, iat: n3, timeoutKey: f2, timeAcquired: Date.now(), timeRefreshed: Date.now() })), [4, i2(30)];
            case 4:
              return r3.sent(), null !== (d2 = l2.getItem(u2)) && (d2 = JSON.parse(d2)).id === this.id && d2.iat === n3 ? (this.acquiredIatSet.add(n3), this.refreshLockWhileAcquired(u2, n3), [2, true]) : [3, 7];
            case 5:
              return e3.lockCorrector(), [4, this.waitForSomethingToChange(s2)];
            case 6:
              r3.sent(), r3.label = 7;
            case 7:
              return n3 = Date.now() + a2(4), [3, 1];
            case 8:
              return [2, false];
          }
        });
      });
    }, e3.prototype.refreshLockWhileAcquired = function(e4, t3) {
      return n2(this, void 0, void 0, function() {
        var o3 = this;
        return r2(this, function(i3) {
          return setTimeout(function() {
            return n2(o3, void 0, void 0, function() {
              var n3, o4;
              return r2(this, function(r3) {
                switch (r3.label) {
                  case 0:
                    return [4, Oo.default().lock(t3)];
                  case 1:
                    return r3.sent(), this.acquiredIatSet.has(t3) ? (n3 = window.localStorage, null === (o4 = n3.getItem(e4)) ? (Oo.default().unlock(t3), [2]) : ((o4 = JSON.parse(o4)).timeRefreshed = Date.now(), n3.setItem(e4, JSON.stringify(o4)), Oo.default().unlock(t3), this.refreshLockWhileAcquired(e4, t3), [2])) : (Oo.default().unlock(t3), [2]);
                }
              });
            });
          }, 1e3), [2];
        });
      });
    }, e3.prototype.waitForSomethingToChange = function(t3) {
      return n2(this, void 0, void 0, function() {
        return r2(this, function(n3) {
          switch (n3.label) {
            case 0:
              return [4, new Promise(function(n4) {
                var r3 = false, o3 = Date.now(), i3 = false;
                function a3() {
                  if (i3 || (window.removeEventListener("storage", a3), e3.removeFromWaiting(a3), clearTimeout(c3), i3 = true), !r3) {
                    r3 = true;
                    var t4 = 50 - (Date.now() - o3);
                    t4 > 0 ? setTimeout(n4, t4) : n4();
                  }
                }
                window.addEventListener("storage", a3), e3.addToWaiting(a3);
                var c3 = setTimeout(a3, Math.max(0, t3 - Date.now()));
              })];
            case 1:
              return n3.sent(), [2];
          }
        });
      });
    }, e3.addToWaiting = function(t3) {
      this.removeFromWaiting(t3), void 0 !== e3.waiters && e3.waiters.push(t3);
    }, e3.removeFromWaiting = function(t3) {
      void 0 !== e3.waiters && (e3.waiters = e3.waiters.filter(function(e4) {
        return e4 !== t3;
      }));
    }, e3.notifyWaiters = function() {
      void 0 !== e3.waiters && e3.waiters.slice().forEach(function(e4) {
        return e4();
      });
    }, e3.prototype.releaseLock = function(e4) {
      return n2(this, void 0, void 0, function() {
        return r2(this, function(t3) {
          switch (t3.label) {
            case 0:
              return [4, this.releaseLock__private__(e4)];
            case 1:
              return [2, t3.sent()];
          }
        });
      });
    }, e3.prototype.releaseLock__private__ = function(t3) {
      return n2(this, void 0, void 0, function() {
        var n3, i3, a3;
        return r2(this, function(r3) {
          switch (r3.label) {
            case 0:
              return n3 = window.localStorage, i3 = o2 + "-" + t3, null === (a3 = n3.getItem(i3)) ? [2] : (a3 = JSON.parse(a3)).id !== this.id ? [3, 2] : [4, Oo.default().lock(a3.iat)];
            case 1:
              r3.sent(), this.acquiredIatSet.delete(a3.iat), n3.removeItem(i3), Oo.default().unlock(a3.iat), e3.notifyWaiters(), r3.label = 2;
            case 2:
              return [2];
          }
        });
      });
    }, e3.lockCorrector = function() {
      for (var t3 = Date.now() - 5e3, n3 = window.localStorage, r3 = Object.keys(n3), i3 = false, a3 = 0; a3 < r3.length; a3++) {
        var c3 = r3[a3];
        if (c3.includes(o2)) {
          var s2 = n3.getItem(c3);
          null !== s2 && (void 0 === (s2 = JSON.parse(s2)).timeRefreshed && s2.timeAcquired < t3 || void 0 !== s2.timeRefreshed && s2.timeRefreshed < t3) && (n3.removeItem(c3), i3 = true);
        }
      }
      i3 && e3.notifyWaiters();
    }, e3.waiters = void 0, e3;
  }();
  t2.default = c2;
}));
var xo = { timeoutInSeconds: 60 };
var Co = ["login_required", "consent_required", "interaction_required", "account_selection_required", "access_denied"];
var Ro = { name: "auth0-spa-js", version: "1.17.0" };
var Lo = function(e2) {
  function n2(t2, r2) {
    var o2 = e2.call(this, r2) || this;
    return o2.error = t2, o2.error_description = r2, Object.setPrototypeOf(o2, n2.prototype), o2;
  }
  return t(n2, e2), n2.fromPayload = function(e3) {
    return new n2(e3.error, e3.error_description);
  }, n2;
}(Error);
var Fo = function(e2) {
  function n2(t2, r2, o2, i2) {
    void 0 === i2 && (i2 = null);
    var a2 = e2.call(this, t2, r2) || this;
    return a2.state = o2, a2.appState = i2, Object.setPrototypeOf(a2, n2.prototype), a2;
  }
  return t(n2, e2), n2;
}(Lo);
var jo = function(e2) {
  function n2() {
    var t2 = e2.call(this, "timeout", "Timeout") || this;
    return Object.setPrototypeOf(t2, n2.prototype), t2;
  }
  return t(n2, e2), n2;
}(Lo);
var Uo = function(e2) {
  function n2(t2) {
    var r2 = e2.call(this) || this;
    return r2.popup = t2, Object.setPrototypeOf(r2, n2.prototype), r2;
  }
  return t(n2, e2), n2;
}(jo);
var Ko = function(e2) {
  function n2(t2) {
    var r2 = e2.call(this, "cancelled", "Popup closed") || this;
    return r2.popup = t2, Object.setPrototypeOf(r2, n2.prototype), r2;
  }
  return t(n2, e2), n2;
}(Lo);
var Ao = function(e2) {
  return new Promise(function(t2, n2) {
    var r2, o2 = setInterval(function() {
      e2.popup && e2.popup.closed && (clearInterval(o2), clearTimeout(i2), window.removeEventListener("message", r2, false), n2(new Ko(e2.popup)));
    }, 1e3), i2 = setTimeout(function() {
      clearInterval(o2), n2(new Uo(e2.popup)), window.removeEventListener("message", r2, false);
    }, 1e3 * (e2.timeoutInSeconds || 60));
    r2 = function(a2) {
      if (a2.data && "authorization_response" === a2.data.type) {
        if (clearTimeout(i2), clearInterval(o2), window.removeEventListener("message", r2, false), e2.popup.close(), a2.data.response.error)
          return n2(Lo.fromPayload(a2.data.response));
        t2(a2.data.response);
      }
    }, window.addEventListener("message", r2);
  });
};
var Po = function() {
  return window.crypto || window.msCrypto;
};
var Wo = function() {
  var e2 = Po();
  return e2.subtle || e2.webkitSubtle;
};
var Zo = function() {
  var e2 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_~.", t2 = "";
  return Array.from(Po().getRandomValues(new Uint8Array(43))).forEach(function(n2) {
    return t2 += e2[n2 % e2.length];
  }), t2;
};
var Vo = function(e2) {
  return btoa(e2);
};
var Xo = function(e2) {
  return Object.keys(e2).filter(function(t2) {
    return void 0 !== e2[t2];
  }).map(function(t2) {
    return encodeURIComponent(t2) + "=" + encodeURIComponent(e2[t2]);
  }).join("&");
};
var No = function(e2) {
  return o(void 0, void 0, void 0, function() {
    var t2;
    return i(this, function(n2) {
      switch (n2.label) {
        case 0:
          return t2 = Wo().digest({ name: "SHA-256" }, new TextEncoder().encode(e2)), window.msCrypto ? [2, new Promise(function(e3, n3) {
            t2.oncomplete = function(t3) {
              e3(t3.target.result);
            }, t2.onerror = function(e4) {
              n3(e4.error);
            }, t2.onabort = function() {
              n3("The digest operation was aborted");
            };
          })] : [4, t2];
        case 1:
          return [2, n2.sent()];
      }
    });
  });
};
var Do = function(e2) {
  return function(e3) {
    return decodeURIComponent(atob(e3).split("").map(function(e4) {
      return "%" + ("00" + e4.charCodeAt(0).toString(16)).slice(-2);
    }).join(""));
  }(e2.replace(/_/g, "/").replace(/-/g, "+"));
};
var zo = function(e2) {
  var t2 = new Uint8Array(e2);
  return function(e3) {
    var t3 = { "+": "-", "/": "_", "=": "" };
    return e3.replace(/[+/=]/g, function(e4) {
      return t3[e4];
    });
  }(window.btoa(String.fromCharCode.apply(String, c([], a(Array.from(t2))))));
};
var Jo = function(e2, t2) {
  return o(void 0, void 0, void 0, function() {
    var n2, r2;
    return i(this, function(o2) {
      switch (o2.label) {
        case 0:
          return [4, (i2 = e2, a2 = t2, a2 = a2 || {}, new Promise(function(e3, t3) {
            var n3 = new XMLHttpRequest(), r3 = [], o3 = [], c2 = {}, s2 = function() {
              return { ok: 2 == (n3.status / 100 | 0), statusText: n3.statusText, status: n3.status, url: n3.responseURL, text: function() {
                return Promise.resolve(n3.responseText);
              }, json: function() {
                return Promise.resolve(n3.responseText).then(JSON.parse);
              }, blob: function() {
                return Promise.resolve(new Blob([n3.response]));
              }, clone: s2, headers: { keys: function() {
                return r3;
              }, entries: function() {
                return o3;
              }, get: function(e4) {
                return c2[e4.toLowerCase()];
              }, has: function(e4) {
                return e4.toLowerCase() in c2;
              } } };
            };
            for (var u2 in n3.open(a2.method || "get", i2, true), n3.onload = function() {
              n3.getAllResponseHeaders().replace(/^(.*?):[^\S\n]*([\s\S]*?)$/gm, function(e4, t4, n4) {
                r3.push(t4 = t4.toLowerCase()), o3.push([t4, n4]), c2[t4] = c2[t4] ? c2[t4] + "," + n4 : n4;
              }), e3(s2());
            }, n3.onerror = t3, n3.withCredentials = "include" == a2.credentials, a2.headers)
              n3.setRequestHeader(u2, a2.headers[u2]);
            n3.send(a2.body || null);
          }))];
        case 1:
          return n2 = o2.sent(), r2 = { ok: n2.ok }, [4, n2.json()];
        case 2:
          return [2, (r2.json = o2.sent(), r2)];
      }
      var i2, a2;
    });
  });
};
var Yo = function(e2, t2, n2) {
  return o(void 0, void 0, void 0, function() {
    var r2, o2;
    return i(this, function(i2) {
      return r2 = new AbortController(), t2.signal = r2.signal, [2, Promise.race([Jo(e2, t2), new Promise(function(e3, t3) {
        o2 = setTimeout(function() {
          r2.abort(), t3(new Error("Timeout when executing 'fetch'"));
        }, n2);
      })]).finally(function() {
        clearTimeout(o2);
      })];
    });
  });
};
var Go = function(e2, t2, n2, r2, a2, c2, s2) {
  return o(void 0, void 0, void 0, function() {
    return i(this, function(o2) {
      return [2, (i2 = { auth: { audience: t2, scope: n2 }, timeout: a2, fetchUrl: e2, fetchOptions: r2, useFormData: s2 }, u2 = c2, new Promise(function(e3, t3) {
        var n3 = new MessageChannel();
        n3.port1.onmessage = function(n4) {
          n4.data.error ? t3(new Error(n4.data.error)) : e3(n4.data);
        }, u2.postMessage(i2, [n3.port2]);
      }))];
      var i2, u2;
    });
  });
};
var Bo = function(e2, t2, n2, r2, a2, c2, s2) {
  return void 0 === s2 && (s2 = 1e4), o(void 0, void 0, void 0, function() {
    return i(this, function(o2) {
      return a2 ? [2, Go(e2, t2, n2, r2, s2, a2, c2)] : [2, Yo(e2, r2, s2)];
    });
  });
};
function Mo(e2, t2, n2, a2, c2, s2, u2) {
  return o(this, void 0, void 0, function() {
    var o2, l2, f2, d2, h2, p2, y2, v2;
    return i(this, function(i2) {
      switch (i2.label) {
        case 0:
          o2 = null, f2 = 0, i2.label = 1;
        case 1:
          if (!(f2 < 3))
            return [3, 6];
          i2.label = 2;
        case 2:
          return i2.trys.push([2, 4, , 5]), [4, Bo(e2, n2, a2, c2, s2, u2, t2)];
        case 3:
          return l2 = i2.sent(), o2 = null, [3, 6];
        case 4:
          return d2 = i2.sent(), o2 = d2, [3, 5];
        case 5:
          return f2++, [3, 1];
        case 6:
          if (o2)
            throw o2.message = o2.message || "Failed to fetch", o2;
          if (h2 = l2.json, p2 = h2.error, y2 = h2.error_description, v2 = r(h2, ["error", "error_description"]), !l2.ok)
            throw new Lo(p2 || "request_error", y2 || "HTTP error. Unable to fetch " + e2);
          return [2, v2];
      }
    });
  });
}
function Ho(e2, t2) {
  var n2 = e2.baseUrl, a2 = e2.timeout, c2 = e2.audience, s2 = e2.scope, u2 = e2.auth0Client, l2 = e2.useFormData, f2 = r(e2, ["baseUrl", "timeout", "audience", "scope", "auth0Client", "useFormData"]);
  return o(this, void 0, void 0, function() {
    var e3;
    return i(this, function(r2) {
      switch (r2.label) {
        case 0:
          return e3 = l2 ? Xo(f2) : JSON.stringify(f2), [4, Mo(n2 + "/oauth/token", a2, c2 || "default", s2, { method: "POST", body: e3, headers: { "Content-Type": l2 ? "application/x-www-form-urlencoded" : "application/json", "Auth0-Client": btoa(JSON.stringify(u2 || Ro)) } }, t2, l2)];
        case 1:
          return [2, r2.sent()];
      }
    });
  });
}
var qo = function(e2) {
  return Array.from(new Set(e2));
};
var Qo = function() {
  for (var e2 = [], t2 = 0; t2 < arguments.length; t2++)
    e2[t2] = arguments[t2];
  return qo(e2.join(" ").trim().split(/\s+/)).join(" ");
};
var $o = function() {
  function e2(e3, t2) {
    void 0 === t2 && (t2 = "@@auth0spajs@@"), this.prefix = t2, this.client_id = e3.client_id, this.scope = e3.scope, this.audience = e3.audience;
  }
  return e2.prototype.toKey = function() {
    return this.prefix + "::" + this.client_id + "::" + this.audience + "::" + this.scope;
  }, e2.fromKey = function(t2) {
    var n2 = a(t2.split("::"), 4), r2 = n2[0], o2 = n2[1], i2 = n2[2];
    return new e2({ client_id: o2, scope: n2[3], audience: i2 }, r2);
  }, e2.fromCacheEntry = function(t2) {
    return new e2({ scope: t2.scope, audience: t2.audience, client_id: t2.client_id });
  }, e2;
}();
var ei = function() {
  function e2() {
  }
  return e2.prototype.set = function(e3, t2) {
    localStorage.setItem(e3, JSON.stringify(t2));
  }, e2.prototype.get = function(e3) {
    var t2 = window.localStorage.getItem(e3);
    if (t2)
      try {
        return JSON.parse(t2);
      } catch (e4) {
        return;
      }
  }, e2.prototype.remove = function(e3) {
    localStorage.removeItem(e3);
  }, e2.prototype.allKeys = function() {
    return Object.keys(window.localStorage).filter(function(e3) {
      return e3.startsWith("@@auth0spajs@@");
    });
  }, e2;
}();
var ti = function() {
  var e2;
  this.enclosedCache = (e2 = {}, { set: function(t2, n2) {
    e2[t2] = n2;
  }, get: function(t2) {
    var n2 = e2[t2];
    if (n2)
      return n2;
  }, remove: function(t2) {
    delete e2[t2];
  }, allKeys: function() {
    return Object.keys(e2);
  } });
};
var ni = function() {
  function e2(e3, t2) {
    this.cache = e3, this.clientId = t2, this.manifestKey = this.createManifestKeyFrom(t2);
  }
  return e2.prototype.add = function(e3) {
    var t2;
    return o(this, void 0, void 0, function() {
      var n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return r2 = Set.bind, [4, this.cache.get(this.manifestKey)];
          case 1:
            return (n2 = new (r2.apply(Set, [void 0, (null === (t2 = o2.sent()) || void 0 === t2 ? void 0 : t2.keys) || []]))()).add(e3), [4, this.cache.set(this.manifestKey, { keys: c([], a(n2)) })];
          case 2:
            return o2.sent(), [2];
        }
      });
    });
  }, e2.prototype.remove = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, n2;
      return i(this, function(r2) {
        switch (r2.label) {
          case 0:
            return [4, this.cache.get(this.manifestKey)];
          case 1:
            return (t2 = r2.sent()) ? ((n2 = new Set(t2.keys)).delete(e3), n2.size > 0 ? [4, this.cache.set(this.manifestKey, { keys: c([], a(n2)) })] : [3, 3]) : [3, 5];
          case 2:
            return [2, r2.sent()];
          case 3:
            return [4, this.cache.remove(this.manifestKey)];
          case 4:
            return [2, r2.sent()];
          case 5:
            return [2];
        }
      });
    });
  }, e2.prototype.get = function() {
    return this.cache.get(this.manifestKey);
  }, e2.prototype.clear = function() {
    return this.cache.remove(this.manifestKey);
  }, e2.prototype.createManifestKeyFrom = function(e3) {
    return "@@auth0spajs@@::" + e3;
  }, e2;
}();
var ri = function() {
  function e2(e3, t2) {
    this.cache = e3, e3.allKeys || (this.keyManifest = new ni(this.cache, t2));
  }
  return e2.prototype.get = function(e3, t2) {
    var n2;
    return void 0 === t2 && (t2 = 0), o(this, void 0, void 0, function() {
      var r2, o2, a2, c2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return [4, this.cache.get(e3.toKey())];
          case 1:
            return (r2 = i2.sent()) ? [3, 4] : [4, this.getCacheKeys()];
          case 2:
            return (o2 = i2.sent()) ? (a2 = this.matchExistingCacheKey(e3, o2), [4, this.cache.get(a2)]) : [2];
          case 3:
            r2 = i2.sent(), i2.label = 4;
          case 4:
            return r2 ? (c2 = Math.floor(Date.now() / 1e3), r2.expiresAt - t2 < c2 ? r2.body.refresh_token ? (r2.body = { refresh_token: r2.body.refresh_token }, [4, this.cache.set(e3.toKey(), r2)]) : [3, 6] : [3, 9]) : [2];
          case 5:
            return i2.sent(), [2, r2.body];
          case 6:
            return [4, this.cache.remove(e3.toKey())];
          case 7:
            return i2.sent(), [4, null === (n2 = this.keyManifest) || void 0 === n2 ? void 0 : n2.remove(e3.toKey())];
          case 8:
            return i2.sent(), [2];
          case 9:
            return [2, r2.body];
        }
      });
    });
  }, e2.prototype.set = function(e3) {
    var t2;
    return o(this, void 0, void 0, function() {
      var n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return n2 = new $o({ client_id: e3.client_id, scope: e3.scope, audience: e3.audience }), r2 = this.wrapCacheEntry(e3), [4, this.cache.set(n2.toKey(), r2)];
          case 1:
            return o2.sent(), [4, null === (t2 = this.keyManifest) || void 0 === t2 ? void 0 : t2.add(n2.toKey())];
          case 2:
            return o2.sent(), [2];
        }
      });
    });
  }, e2.prototype.clear = function() {
    var e3;
    return o(this, void 0, void 0, function() {
      var t2, n2 = this;
      return i(this, function(r2) {
        switch (r2.label) {
          case 0:
            return [4, this.getCacheKeys()];
          case 1:
            return (t2 = r2.sent()) ? (t2.forEach(function(e4) {
              return o(n2, void 0, void 0, function() {
                return i(this, function(t3) {
                  switch (t3.label) {
                    case 0:
                      return [4, this.cache.remove(e4)];
                    case 1:
                      return t3.sent(), [2];
                  }
                });
              });
            }), [4, null === (e3 = this.keyManifest) || void 0 === e3 ? void 0 : e3.clear()]) : [2];
          case 2:
            return r2.sent(), [2];
        }
      });
    });
  }, e2.prototype.clearSync = function() {
    var e3 = this, t2 = this.cache.allKeys();
    t2 && t2.forEach(function(t3) {
      e3.cache.remove(t3);
    });
  }, e2.prototype.wrapCacheEntry = function(e3) {
    var t2 = Math.floor(Date.now() / 1e3) + e3.expires_in;
    return { body: e3, expiresAt: Math.min(t2, e3.decodedToken.claims.exp) };
  }, e2.prototype.getCacheKeys = function() {
    var e3;
    return o(this, void 0, void 0, function() {
      var t2;
      return i(this, function(n2) {
        switch (n2.label) {
          case 0:
            return this.keyManifest ? [4, this.keyManifest.get()] : [3, 2];
          case 1:
            return t2 = null === (e3 = n2.sent()) || void 0 === e3 ? void 0 : e3.keys, [3, 4];
          case 2:
            return [4, this.cache.allKeys()];
          case 3:
            t2 = n2.sent(), n2.label = 4;
          case 4:
            return [2, t2];
        }
      });
    });
  }, e2.prototype.matchExistingCacheKey = function(e3, t2) {
    return t2.filter(function(t3) {
      var n2 = $o.fromKey(t3), r2 = new Set(n2.scope && n2.scope.split(" ")), o2 = e3.scope.split(" "), i2 = n2.scope && o2.reduce(function(e4, t4) {
        return e4 && r2.has(t4);
      }, true);
      return "@@auth0spajs@@" === n2.prefix && n2.client_id === e3.client_id && n2.audience === e3.audience && i2;
    })[0];
  }, e2;
}();
var oi = function() {
  function e2(e3) {
    this.storage = e3, this.transaction = this.storage.get("a0.spajs.txs");
  }
  return e2.prototype.create = function(e3) {
    this.transaction = e3, this.storage.save("a0.spajs.txs", e3, { daysUntilExpire: 1 });
  }, e2.prototype.get = function() {
    return this.transaction;
  }, e2.prototype.remove = function() {
    delete this.transaction, this.storage.remove("a0.spajs.txs");
  }, e2;
}();
var ii = function(e2) {
  return "number" == typeof e2;
};
var ai = ["iss", "aud", "exp", "nbf", "iat", "jti", "azp", "nonce", "auth_time", "at_hash", "c_hash", "acr", "amr", "sub_jwk", "cnf", "sip_from_tag", "sip_date", "sip_callid", "sip_cseq_num", "sip_via_branch", "orig", "dest", "mky", "events", "toe", "txn", "rph", "sid", "vot", "vtm"];
var ci = function(e2) {
  if (!e2.id_token)
    throw new Error("ID token is required but missing");
  var t2 = function(e3) {
    var t3 = e3.split("."), n3 = a(t3, 3), r3 = n3[0], o3 = n3[1], i3 = n3[2];
    if (3 !== t3.length || !r3 || !o3 || !i3)
      throw new Error("ID token could not be decoded");
    var c3 = JSON.parse(Do(o3)), s2 = { __raw: e3 }, u2 = {};
    return Object.keys(c3).forEach(function(e4) {
      s2[e4] = c3[e4], ai.includes(e4) || (u2[e4] = c3[e4]);
    }), { encoded: { header: r3, payload: o3, signature: i3 }, header: JSON.parse(Do(r3)), claims: s2, user: u2 };
  }(e2.id_token);
  if (!t2.claims.iss)
    throw new Error("Issuer (iss) claim must be a string present in the ID token");
  if (t2.claims.iss !== e2.iss)
    throw new Error('Issuer (iss) claim mismatch in the ID token; expected "' + e2.iss + '", found "' + t2.claims.iss + '"');
  if (!t2.user.sub)
    throw new Error("Subject (sub) claim must be a string present in the ID token");
  if ("RS256" !== t2.header.alg)
    throw new Error('Signature algorithm of "' + t2.header.alg + '" is not supported. Expected the ID token to be signed with "RS256".');
  if (!t2.claims.aud || "string" != typeof t2.claims.aud && !Array.isArray(t2.claims.aud))
    throw new Error("Audience (aud) claim must be a string or array of strings present in the ID token");
  if (Array.isArray(t2.claims.aud)) {
    if (!t2.claims.aud.includes(e2.aud))
      throw new Error('Audience (aud) claim mismatch in the ID token; expected "' + e2.aud + '" but was not one of "' + t2.claims.aud.join(", ") + '"');
    if (t2.claims.aud.length > 1) {
      if (!t2.claims.azp)
        throw new Error("Authorized Party (azp) claim must be a string present in the ID token when Audience (aud) claim has multiple values");
      if (t2.claims.azp !== e2.aud)
        throw new Error('Authorized Party (azp) claim mismatch in the ID token; expected "' + e2.aud + '", found "' + t2.claims.azp + '"');
    }
  } else if (t2.claims.aud !== e2.aud)
    throw new Error('Audience (aud) claim mismatch in the ID token; expected "' + e2.aud + '" but found "' + t2.claims.aud + '"');
  if (e2.nonce) {
    if (!t2.claims.nonce)
      throw new Error("Nonce (nonce) claim must be a string present in the ID token");
    if (t2.claims.nonce !== e2.nonce)
      throw new Error('Nonce (nonce) claim mismatch in the ID token; expected "' + e2.nonce + '", found "' + t2.claims.nonce + '"');
  }
  if (e2.max_age && !ii(t2.claims.auth_time))
    throw new Error("Authentication Time (auth_time) claim must be a number present in the ID token when Max Age (max_age) is specified");
  if (!ii(t2.claims.exp))
    throw new Error("Expiration Time (exp) claim must be a number present in the ID token");
  if (!ii(t2.claims.iat))
    throw new Error("Issued At (iat) claim must be a number present in the ID token");
  var n2 = e2.leeway || 60, r2 = new Date(Date.now()), o2 = /* @__PURE__ */ new Date(0), i2 = /* @__PURE__ */ new Date(0), c2 = /* @__PURE__ */ new Date(0);
  if (c2.setUTCSeconds(parseInt(t2.claims.auth_time) + e2.max_age + n2), o2.setUTCSeconds(t2.claims.exp + n2), i2.setUTCSeconds(t2.claims.nbf - n2), r2 > o2)
    throw new Error("Expiration Time (exp) claim error in the ID token; current time (" + r2 + ") is after expiration time (" + o2 + ")");
  if (ii(t2.claims.nbf) && r2 < i2)
    throw new Error("Not Before time (nbf) claim in the ID token indicates that this token can't be used just yet. Currrent time (" + r2 + ") is before " + i2);
  if (ii(t2.claims.auth_time) && r2 > c2)
    throw new Error("Authentication Time (auth_time) claim in the ID token indicates that too much time has passed since the last end-user authentication. Currrent time (" + r2 + ") is after last auth at " + c2);
  if (e2.organizationId) {
    if (!t2.claims.org_id)
      throw new Error("Organization ID (org_id) claim must be a string present in the ID token");
    if (e2.organizationId !== t2.claims.org_id)
      throw new Error('Organization ID (org_id) claim mismatch in the ID token; expected "' + e2.organizationId + '", found "' + t2.claims.org_id + '"');
  }
  return t2;
};
var si = l(function(e2, t2) {
  var n2 = s && s.__assign || function() {
    return (n2 = Object.assign || function(e3) {
      for (var t3, n3 = 1, r3 = arguments.length; n3 < r3; n3++)
        for (var o3 in t3 = arguments[n3])
          Object.prototype.hasOwnProperty.call(t3, o3) && (e3[o3] = t3[o3]);
      return e3;
    }).apply(this, arguments);
  };
  function r2(e3, t3) {
    if (!t3)
      return "";
    var n3 = "; " + e3;
    return true === t3 ? n3 : n3 + "=" + t3;
  }
  function o2(e3, t3, n3) {
    return encodeURIComponent(e3).replace(/%(23|24|26|2B|5E|60|7C)/g, decodeURIComponent).replace(/\(/g, "%28").replace(/\)/g, "%29") + "=" + encodeURIComponent(t3).replace(/%(23|24|26|2B|3A|3C|3E|3D|2F|3F|40|5B|5D|5E|60|7B|7D|7C)/g, decodeURIComponent) + function(e4) {
      if ("number" == typeof e4.expires) {
        var t4 = /* @__PURE__ */ new Date();
        t4.setMilliseconds(t4.getMilliseconds() + 864e5 * e4.expires), e4.expires = t4;
      }
      return r2("Expires", e4.expires ? e4.expires.toUTCString() : "") + r2("Domain", e4.domain) + r2("Path", e4.path) + r2("Secure", e4.secure) + r2("SameSite", e4.sameSite);
    }(n3);
  }
  function i2(e3) {
    for (var t3 = {}, n3 = e3 ? e3.split("; ") : [], r3 = /(%[\dA-F]{2})+/gi, o3 = 0; o3 < n3.length; o3++) {
      var i3 = n3[o3].split("="), a3 = i3.slice(1).join("=");
      '"' === a3.charAt(0) && (a3 = a3.slice(1, -1));
      try {
        t3[i3[0].replace(r3, decodeURIComponent)] = a3.replace(r3, decodeURIComponent);
      } catch (e4) {
      }
    }
    return t3;
  }
  function a2() {
    return i2(document.cookie);
  }
  function c2(e3, t3, r3) {
    document.cookie = o2(e3, t3, n2({ path: "/" }, r3));
  }
  t2.__esModule = true, t2.encode = o2, t2.parse = i2, t2.getAll = a2, t2.get = function(e3) {
    return a2()[e3];
  }, t2.set = c2, t2.remove = function(e3, t3) {
    c2(e3, "", n2(n2({}, t3), { expires: -1 }));
  };
});
u(si), si.encode, si.parse, si.getAll;
var ui = si.get;
var li = si.set;
var fi = si.remove;
var di = { get: function(e2) {
  var t2 = ui(e2);
  if (void 0 !== t2)
    return JSON.parse(t2);
}, save: function(e2, t2, n2) {
  var r2 = {};
  "https:" === window.location.protocol && (r2 = { secure: true, sameSite: "none" }), r2.expires = n2.daysUntilExpire, li(e2, JSON.stringify(t2), r2);
}, remove: function(e2) {
  fi(e2);
} };
var hi = { get: function(e2) {
  var t2 = di.get(e2);
  return t2 || di.get("_legacy_" + e2);
}, save: function(e2, t2, n2) {
  var r2 = {};
  "https:" === window.location.protocol && (r2 = { secure: true }), r2.expires = n2.daysUntilExpire, li("_legacy_" + e2, JSON.stringify(t2), r2), di.save(e2, t2, n2);
}, remove: function(e2) {
  di.remove(e2), di.remove("_legacy_" + e2);
} };
var pi = { get: function(e2) {
  if ("undefined" != typeof sessionStorage) {
    var t2 = sessionStorage.getItem(e2);
    if (void 0 !== t2)
      return JSON.parse(t2);
  }
}, save: function(e2, t2) {
  sessionStorage.setItem(e2, JSON.stringify(t2));
}, remove: function(e2) {
  sessionStorage.removeItem(e2);
} };
function yi(e2, t2, n2) {
  var r2 = void 0 === t2 ? null : t2, o2 = function(e3, t3) {
    var n3 = atob(e3);
    if (t3) {
      for (var r3 = new Uint8Array(n3.length), o3 = 0, i3 = n3.length; o3 < i3; ++o3)
        r3[o3] = n3.charCodeAt(o3);
      return String.fromCharCode.apply(null, new Uint16Array(r3.buffer));
    }
    return n3;
  }(e2, void 0 !== n2 && n2), i2 = o2.indexOf("\n", 10) + 1, a2 = o2.substring(i2) + (r2 ? "//# sourceMappingURL=" + r2 : ""), c2 = new Blob([a2], { type: "application/javascript" });
  return URL.createObjectURL(c2);
}
var vi;
var mi;
var gi;
var bi;
var wi = (vi = "Lyogcm9sbHVwLXBsdWdpbi13ZWItd29ya2VyLWxvYWRlciAqLwohZnVuY3Rpb24oKXsidXNlIHN0cmljdCI7Ci8qISAqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKgogICAgQ29weXJpZ2h0IChjKSBNaWNyb3NvZnQgQ29ycG9yYXRpb24uCgogICAgUGVybWlzc2lvbiB0byB1c2UsIGNvcHksIG1vZGlmeSwgYW5kL29yIGRpc3RyaWJ1dGUgdGhpcyBzb2Z0d2FyZSBmb3IgYW55CiAgICBwdXJwb3NlIHdpdGggb3Igd2l0aG91dCBmZWUgaXMgaGVyZWJ5IGdyYW50ZWQuCgogICAgVEhFIFNPRlRXQVJFIElTIFBST1ZJREVEICJBUyBJUyIgQU5EIFRIRSBBVVRIT1IgRElTQ0xBSU1TIEFMTCBXQVJSQU5USUVTIFdJVEgKICAgIFJFR0FSRCBUTyBUSElTIFNPRlRXQVJFIElOQ0xVRElORyBBTEwgSU1QTElFRCBXQVJSQU5USUVTIE9GIE1FUkNIQU5UQUJJTElUWQogICAgQU5EIEZJVE5FU1MuIElOIE5PIEVWRU5UIFNIQUxMIFRIRSBBVVRIT1IgQkUgTElBQkxFIEZPUiBBTlkgU1BFQ0lBTCwgRElSRUNULAogICAgSU5ESVJFQ1QsIE9SIENPTlNFUVVFTlRJQUwgREFNQUdFUyBPUiBBTlkgREFNQUdFUyBXSEFUU09FVkVSIFJFU1VMVElORyBGUk9NCiAgICBMT1NTIE9GIFVTRSwgREFUQSBPUiBQUk9GSVRTLCBXSEVUSEVSIElOIEFOIEFDVElPTiBPRiBDT05UUkFDVCwgTkVHTElHRU5DRSBPUgogICAgT1RIRVIgVE9SVElPVVMgQUNUSU9OLCBBUklTSU5HIE9VVCBPRiBPUiBJTiBDT05ORUNUSU9OIFdJVEggVEhFIFVTRSBPUgogICAgUEVSRk9STUFOQ0UgT0YgVEhJUyBTT0ZUV0FSRS4KICAgICoqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqKioqICovdmFyIGU9ZnVuY3Rpb24oKXtyZXR1cm4oZT1PYmplY3QuYXNzaWdufHxmdW5jdGlvbihlKXtmb3IodmFyIHIsdD0xLG49YXJndW1lbnRzLmxlbmd0aDt0PG47dCsrKWZvcih2YXIgbyBpbiByPWFyZ3VtZW50c1t0XSlPYmplY3QucHJvdG90eXBlLmhhc093blByb3BlcnR5LmNhbGwocixvKSYmKGVbb109cltvXSk7cmV0dXJuIGV9KS5hcHBseSh0aGlzLGFyZ3VtZW50cyl9O2Z1bmN0aW9uIHIoZSxyLHQsbil7cmV0dXJuIG5ldyh0fHwodD1Qcm9taXNlKSkoKGZ1bmN0aW9uKG8sYSl7ZnVuY3Rpb24gcyhlKXt0cnl7dShuLm5leHQoZSkpfWNhdGNoKGUpe2EoZSl9fWZ1bmN0aW9uIGkoZSl7dHJ5e3Uobi50aHJvdyhlKSl9Y2F0Y2goZSl7YShlKX19ZnVuY3Rpb24gdShlKXt2YXIgcjtlLmRvbmU/byhlLnZhbHVlKToocj1lLnZhbHVlLHIgaW5zdGFuY2VvZiB0P3I6bmV3IHQoKGZ1bmN0aW9uKGUpe2Uocil9KSkpLnRoZW4ocyxpKX11KChuPW4uYXBwbHkoZSxyfHxbXSkpLm5leHQoKSl9KSl9ZnVuY3Rpb24gdChlLHIpe3ZhciB0LG4sbyxhLHM9e2xhYmVsOjAsc2VudDpmdW5jdGlvbigpe2lmKDEmb1swXSl0aHJvdyBvWzFdO3JldHVybiBvWzFdfSx0cnlzOltdLG9wczpbXX07cmV0dXJuIGE9e25leHQ6aSgwKSx0aHJvdzppKDEpLHJldHVybjppKDIpfSwiZnVuY3Rpb24iPT10eXBlb2YgU3ltYm9sJiYoYVtTeW1ib2wuaXRlcmF0b3JdPWZ1bmN0aW9uKCl7cmV0dXJuIHRoaXN9KSxhO2Z1bmN0aW9uIGkoYSl7cmV0dXJuIGZ1bmN0aW9uKGkpe3JldHVybiBmdW5jdGlvbihhKXtpZih0KXRocm93IG5ldyBUeXBlRXJyb3IoIkdlbmVyYXRvciBpcyBhbHJlYWR5IGV4ZWN1dGluZy4iKTtmb3IoO3M7KXRyeXtpZih0PTEsbiYmKG89MiZhWzBdP24ucmV0dXJuOmFbMF0/bi50aHJvd3x8KChvPW4ucmV0dXJuKSYmby5jYWxsKG4pLDApOm4ubmV4dCkmJiEobz1vLmNhbGwobixhWzFdKSkuZG9uZSlyZXR1cm4gbztzd2l0Y2gobj0wLG8mJihhPVsyJmFbMF0sby52YWx1ZV0pLGFbMF0pe2Nhc2UgMDpjYXNlIDE6bz1hO2JyZWFrO2Nhc2UgNDpyZXR1cm4gcy5sYWJlbCsrLHt2YWx1ZTphWzFdLGRvbmU6ITF9O2Nhc2UgNTpzLmxhYmVsKyssbj1hWzFdLGE9WzBdO2NvbnRpbnVlO2Nhc2UgNzphPXMub3BzLnBvcCgpLHMudHJ5cy5wb3AoKTtjb250aW51ZTtkZWZhdWx0OmlmKCEobz1zLnRyeXMsKG89by5sZW5ndGg+MCYmb1tvLmxlbmd0aC0xXSl8fDYhPT1hWzBdJiYyIT09YVswXSkpe3M9MDtjb250aW51ZX1pZigzPT09YVswXSYmKCFvfHxhWzFdPm9bMF0mJmFbMV08b1szXSkpe3MubGFiZWw9YVsxXTticmVha31pZig2PT09YVswXSYmcy5sYWJlbDxvWzFdKXtzLmxhYmVsPW9bMV0sbz1hO2JyZWFrfWlmKG8mJnMubGFiZWw8b1syXSl7cy5sYWJlbD1vWzJdLHMub3BzLnB1c2goYSk7YnJlYWt9b1syXSYmcy5vcHMucG9wKCkscy50cnlzLnBvcCgpO2NvbnRpbnVlfWE9ci5jYWxsKGUscyl9Y2F0Y2goZSl7YT1bNixlXSxuPTB9ZmluYWxseXt0PW89MH1pZig1JmFbMF0pdGhyb3cgYVsxXTtyZXR1cm57dmFsdWU6YVswXT9hWzFdOnZvaWQgMCxkb25lOiEwfX0oW2EsaV0pfX19dmFyIG49e30sbz1mdW5jdGlvbihlLHIpe3JldHVybiBlKyJ8IityfTthZGRFdmVudExpc3RlbmVyKCJtZXNzYWdlIiwoZnVuY3Rpb24oYSl7dmFyIHM9YS5kYXRhLGk9cy50aW1lb3V0LHU9cy5hdXRoLGM9cy5mZXRjaFVybCxmPXMuZmV0Y2hPcHRpb25zLGw9cy51c2VGb3JtRGF0YSxoPWZ1bmN0aW9uKGUscil7dmFyIHQ9ImZ1bmN0aW9uIj09dHlwZW9mIFN5bWJvbCYmZVtTeW1ib2wuaXRlcmF0b3JdO2lmKCF0KXJldHVybiBlO3ZhciBuLG8sYT10LmNhbGwoZSkscz1bXTt0cnl7Zm9yKDsodm9pZCAwPT09cnx8ci0tID4wKSYmIShuPWEubmV4dCgpKS5kb25lOylzLnB1c2gobi52YWx1ZSl9Y2F0Y2goZSl7bz17ZXJyb3I6ZX19ZmluYWxseXt0cnl7biYmIW4uZG9uZSYmKHQ9YS5yZXR1cm4pJiZ0LmNhbGwoYSl9ZmluYWxseXtpZihvKXRocm93IG8uZXJyb3J9fXJldHVybiBzfShhLnBvcnRzLDEpWzBdO3JldHVybiByKHZvaWQgMCx2b2lkIDAsdm9pZCAwLChmdW5jdGlvbigpe3ZhciByLGEscyxwLHksYixkLHYsdyxnO3JldHVybiB0KHRoaXMsKGZ1bmN0aW9uKHQpe3N3aXRjaCh0LmxhYmVsKXtjYXNlIDA6cz0oYT11fHx7fSkuYXVkaWVuY2UscD1hLnNjb3BlLHQubGFiZWw9MTtjYXNlIDE6aWYodC50cnlzLnB1c2goWzEsNywsOF0pLCEoeT1sPyhrPWYuYm9keSxTPW5ldyBVUkxTZWFyY2hQYXJhbXMoayksXz17fSxTLmZvckVhY2goKGZ1bmN0aW9uKGUscil7X1tyXT1lfSkpLF8pOkpTT04ucGFyc2UoZi5ib2R5KSkucmVmcmVzaF90b2tlbiYmInJlZnJlc2hfdG9rZW4iPT09eS5ncmFudF90eXBlKXtpZighKGI9ZnVuY3Rpb24oZSxyKXtyZXR1cm4gbltvKGUscildfShzLHApKSl0aHJvdyBuZXcgRXJyb3IoIlRoZSB3ZWIgd29ya2VyIGlzIG1pc3NpbmcgdGhlIHJlZnJlc2ggdG9rZW4iKTtmLmJvZHk9bD9uZXcgVVJMU2VhcmNoUGFyYW1zKGUoZSh7fSx5KSx7cmVmcmVzaF90b2tlbjpifSkpLnRvU3RyaW5nKCk6SlNPTi5zdHJpbmdpZnkoZShlKHt9LHkpLHtyZWZyZXNoX3Rva2VuOmJ9KSl9ZD12b2lkIDAsImZ1bmN0aW9uIj09dHlwZW9mIEFib3J0Q29udHJvbGxlciYmKGQ9bmV3IEFib3J0Q29udHJvbGxlcixmLnNpZ25hbD1kLnNpZ25hbCksdj12b2lkIDAsdC5sYWJlbD0yO2Nhc2UgMjpyZXR1cm4gdC50cnlzLnB1c2goWzIsNCwsNV0pLFs0LFByb21pc2UucmFjZShbKG09aSxuZXcgUHJvbWlzZSgoZnVuY3Rpb24oZSl7cmV0dXJuIHNldFRpbWVvdXQoZSxtKX0pKSksZmV0Y2goYyxlKHt9LGYpKV0pXTtjYXNlIDM6cmV0dXJuIHY9dC5zZW50KCksWzMsNV07Y2FzZSA0OnJldHVybiB3PXQuc2VudCgpLGgucG9zdE1lc3NhZ2Uoe2Vycm9yOncubWVzc2FnZX0pLFsyXTtjYXNlIDU6cmV0dXJuIHY/WzQsdi5qc29uKCldOihkJiZkLmFib3J0KCksaC5wb3N0TWVzc2FnZSh7ZXJyb3I6IlRpbWVvdXQgd2hlbiBleGVjdXRpbmcgJ2ZldGNoJyJ9KSxbMl0pO2Nhc2UgNjpyZXR1cm4ocj10LnNlbnQoKSkucmVmcmVzaF90b2tlbj8oZnVuY3Rpb24oZSxyLHQpe25bbyhyLHQpXT1lfShyLnJlZnJlc2hfdG9rZW4scyxwKSxkZWxldGUgci5yZWZyZXNoX3Rva2VuKTpmdW5jdGlvbihlLHIpe2RlbGV0ZSBuW28oZSxyKV19KHMscCksaC5wb3N0TWVzc2FnZSh7b2s6di5vayxqc29uOnJ9KSxbMyw4XTtjYXNlIDc6cmV0dXJuIGc9dC5zZW50KCksaC5wb3N0TWVzc2FnZSh7b2s6ITEsanNvbjp7ZXJyb3JfZGVzY3JpcHRpb246Zy5tZXNzYWdlfX0pLFszLDhdO2Nhc2UgODpyZXR1cm5bMl19dmFyIG0sayxTLF99KSl9KSl9KSl9KCk7Cgo=", mi = null, gi = false, function(e2) {
  return bi = bi || yi(vi, mi, gi), new Worker(bi, e2);
});
var Si = {};
var _i = new Eo();
var ki = { memory: function() {
  return new ti().enclosedCache;
}, localstorage: function() {
  return new ei();
} };
var Ii = function(e2) {
  return ki[e2];
};
var Ti = function() {
  return !/Trident.*rv:11\.0/.test(navigator.userAgent);
};
var Oi = function() {
  function e2(e3) {
    var t2, n2, o2;
    if (this.options = e3, "undefined" != typeof window && function() {
      if (!Po())
        throw new Error("For security reasons, `window.crypto` is required to run `auth0-spa-js`.");
      if (void 0 === Wo())
        throw new Error("\n      auth0-spa-js must run on a secure origin. See https://github.com/auth0/auth0-spa-js/blob/master/FAQ.md#why-do-i-get-auth0-spa-js-must-run-on-a-secure-origin for more information.\n    ");
    }(), e3.cache && e3.cacheLocation && console.warn("Both `cache` and `cacheLocation` options have been specified in the Auth0Client configuration; ignoring `cacheLocation` and using `cache`."), e3.cache)
      o2 = e3.cache;
    else {
      if (this.cacheLocation = e3.cacheLocation || "memory", !Ii(this.cacheLocation))
        throw new Error('Invalid cache location "' + this.cacheLocation + '"');
      o2 = Ii(this.cacheLocation)();
    }
    this.cookieStorage = false === e3.legacySameSiteCookie ? di : hi, this.sessionCheckExpiryDays = e3.sessionCheckExpiryDays || 1;
    var i2, a2 = e3.useCookiesForTransactions ? this.cookieStorage : pi;
    this.scope = this.options.scope, this.transactionManager = new oi(a2), this.cacheManager = new ri(o2, this.options.client_id), this.domainUrl = (i2 = this.options.domain, /^https?:\/\//.test(i2) ? i2 : "https://" + i2), this.tokenIssuer = function(e4, t3) {
      return e4 ? e4.startsWith("https://") ? e4 : "https://" + e4 + "/" : t3 + "/";
    }(this.options.issuer, this.domainUrl), this.defaultScope = Qo("openid", void 0 !== (null === (n2 = null === (t2 = this.options) || void 0 === t2 ? void 0 : t2.advancedOptions) || void 0 === n2 ? void 0 : n2.defaultScope) ? this.options.advancedOptions.defaultScope : "openid profile email"), this.options.useRefreshTokens && (this.scope = Qo(this.scope, "offline_access")), "undefined" != typeof window && window.Worker && this.options.useRefreshTokens && "memory" === this.cacheLocation && Ti() && (this.worker = new wi()), this.customOptions = function(e4) {
      return e4.advancedOptions, e4.audience, e4.auth0Client, e4.authorizeTimeoutInSeconds, e4.cacheLocation, e4.client_id, e4.domain, e4.issuer, e4.leeway, e4.max_age, e4.redirect_uri, e4.scope, e4.useRefreshTokens, e4.useCookiesForTransactions, e4.useFormData, r(e4, ["advancedOptions", "audience", "auth0Client", "authorizeTimeoutInSeconds", "cacheLocation", "client_id", "domain", "issuer", "leeway", "max_age", "redirect_uri", "scope", "useRefreshTokens", "useCookiesForTransactions", "useFormData"]);
    }(e3);
  }
  return e2.prototype._url = function(e3) {
    var t2 = encodeURIComponent(btoa(JSON.stringify(this.options.auth0Client || Ro)));
    return "" + this.domainUrl + e3 + "&auth0Client=" + t2;
  }, e2.prototype._getParams = function(e3, t2, o2, i2, a2) {
    var c2 = this.options;
    c2.domain, c2.leeway, c2.useRefreshTokens, c2.useCookiesForTransactions, c2.useFormData, c2.auth0Client, c2.cacheLocation, c2.advancedOptions;
    var s2 = r(c2, ["domain", "leeway", "useRefreshTokens", "useCookiesForTransactions", "useFormData", "auth0Client", "cacheLocation", "advancedOptions"]);
    return n(n(n({}, s2), e3), { scope: Qo(this.defaultScope, this.scope, e3.scope), response_type: "code", response_mode: "query", state: t2, nonce: o2, redirect_uri: a2 || this.options.redirect_uri, code_challenge: i2, code_challenge_method: "S256" });
  }, e2.prototype._authorizeUrl = function(e3) {
    return this._url("/authorize?" + Xo(e3));
  }, e2.prototype._verifyIdToken = function(e3, t2, n2) {
    return ci({ iss: this.tokenIssuer, aud: this.options.client_id, id_token: e3, nonce: t2, organizationId: n2, leeway: this.options.leeway, max_age: this._parseNumber(this.options.max_age) });
  }, e2.prototype._parseNumber = function(e3) {
    return "string" != typeof e3 ? e3 : parseInt(e3, 10) || void 0;
  }, e2.prototype.buildAuthorizeUrl = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, o2, a2, c2, s2, u2, l2, f2, d2, h2, p2, y2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = e3.redirect_uri, o2 = e3.appState, a2 = r(e3, ["redirect_uri", "appState"]), c2 = Vo(Zo()), s2 = Vo(Zo()), u2 = Zo(), [4, No(u2)];
          case 1:
            return l2 = i2.sent(), f2 = zo(l2), d2 = e3.fragment ? "#" + e3.fragment : "", h2 = this._getParams(a2, c2, s2, f2, t2), p2 = this._authorizeUrl(h2), y2 = e3.organization || this.options.organization, this.transactionManager.create(n({ nonce: s2, code_verifier: u2, appState: o2, scope: h2.scope, audience: h2.audience || "default", redirect_uri: h2.redirect_uri }, y2 && { organizationId: y2 })), [2, p2 + d2];
        }
      });
    });
  }, e2.prototype.loginWithPopup = function(e3, t2) {
    return o(this, void 0, void 0, function() {
      var o2, a2, c2, s2, u2, l2, f2, d2, h2, p2, y2, v2, m2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return e3 = e3 || {}, (t2 = t2 || {}).popup || (t2.popup = function(e4) {
              var t3 = window.screenX + (window.innerWidth - 400) / 2, n2 = window.screenY + (window.innerHeight - 600) / 2;
              return window.open(e4, "auth0:authorize:popup", "left=" + t3 + ",top=" + n2 + ",width=400,height=600,resizable,scrollbars=yes,status=1");
            }("")), o2 = r(e3, []), a2 = Vo(Zo()), c2 = Vo(Zo()), s2 = Zo(), [4, No(s2)];
          case 1:
            return u2 = i2.sent(), l2 = zo(u2), f2 = this._getParams(o2, a2, c2, l2, this.options.redirect_uri || window.location.origin), d2 = this._authorizeUrl(n(n({}, f2), { response_mode: "web_message" })), t2.popup.location.href = d2, [4, Ao(n(n({}, t2), { timeoutInSeconds: t2.timeoutInSeconds || this.options.authorizeTimeoutInSeconds || 60 }))];
          case 2:
            if (h2 = i2.sent(), a2 !== h2.state)
              throw new Error("Invalid state");
            return [4, Ho({ audience: f2.audience, scope: f2.scope, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: s2, code: h2.code, grant_type: "authorization_code", redirect_uri: f2.redirect_uri, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData }, this.worker)];
          case 3:
            return p2 = i2.sent(), y2 = e3.organization || this.options.organization, v2 = this._verifyIdToken(p2.id_token, c2, y2), m2 = n(n({}, p2), { decodedToken: v2, scope: f2.scope, audience: f2.audience || "default", client_id: this.options.client_id }), [4, this.cacheManager.set(m2)];
          case 4:
            return i2.sent(), this.cookieStorage.save("auth0.is.authenticated", true, { daysUntilExpire: this.sessionCheckExpiryDays }), [2];
        }
      });
    });
  }, e2.prototype.getUser = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return t2 = e3.audience || this.options.audience || "default", n2 = Qo(this.defaultScope, this.scope, e3.scope), [4, this.cacheManager.get(new $o({ client_id: this.options.client_id, audience: t2, scope: n2 }))];
          case 1:
            return [2, (r2 = o2.sent()) && r2.decodedToken && r2.decodedToken.user];
        }
      });
    });
  }, e2.prototype.getIdTokenClaims = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, r2;
      return i(this, function(o2) {
        switch (o2.label) {
          case 0:
            return t2 = e3.audience || this.options.audience || "default", n2 = Qo(this.defaultScope, this.scope, e3.scope), [4, this.cacheManager.get(new $o({ client_id: this.options.client_id, audience: t2, scope: n2 }))];
          case 1:
            return [2, (r2 = o2.sent()) && r2.decodedToken && r2.decodedToken.claims];
        }
      });
    });
  }, e2.prototype.loginWithRedirect = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, n2, o2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = e3.redirectMethod, n2 = r(e3, ["redirectMethod"]), [4, this.buildAuthorizeUrl(n2)];
          case 1:
            return o2 = i2.sent(), window.location[t2 || "assign"](o2), [2];
        }
      });
    });
  }, e2.prototype.handleRedirectCallback = function(e3) {
    return void 0 === e3 && (e3 = window.location.href), o(this, void 0, void 0, function() {
      var t2, r2, o2, c2, s2, u2, l2, f2, d2, h2, p2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            if (0 === (t2 = e3.split("?").slice(1)).length)
              throw new Error("There are no query params available for parsing.");
            if (r2 = function(e4) {
              e4.indexOf("#") > -1 && (e4 = e4.substr(0, e4.indexOf("#")));
              var t3 = e4.split("&"), n2 = {};
              return t3.forEach(function(e5) {
                var t4 = a(e5.split("="), 2), r3 = t4[0], o3 = t4[1];
                n2[r3] = decodeURIComponent(o3);
              }), n2.expires_in && (n2.expires_in = parseInt(n2.expires_in)), n2;
            }(t2.join("")), o2 = r2.state, c2 = r2.code, s2 = r2.error, u2 = r2.error_description, !(l2 = this.transactionManager.get()) || !l2.code_verifier)
              throw new Error("Invalid state");
            if (this.transactionManager.remove(), s2)
              throw new Fo(s2, u2, o2, l2.appState);
            return f2 = { audience: l2.audience, scope: l2.scope, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: l2.code_verifier, grant_type: "authorization_code", code: c2, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData }, void 0 !== l2.redirect_uri && (f2.redirect_uri = l2.redirect_uri), [4, Ho(f2, this.worker)];
          case 1:
            return d2 = i2.sent(), h2 = this._verifyIdToken(d2.id_token, l2.nonce, l2.organizationId), p2 = n(n({}, d2), { decodedToken: h2, audience: l2.audience, scope: l2.scope, client_id: this.options.client_id }), [4, this.cacheManager.set(p2)];
          case 2:
            return i2.sent(), this.cookieStorage.save("auth0.is.authenticated", true, { daysUntilExpire: this.sessionCheckExpiryDays }), [2, { appState: l2.appState }];
        }
      });
    });
  }, e2.prototype.checkSession = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2;
      return i(this, function(n2) {
        switch (n2.label) {
          case 0:
            if (!this.cookieStorage.get("auth0.is.authenticated"))
              return [2];
            n2.label = 1;
          case 1:
            return n2.trys.push([1, 3, , 4]), [4, this.getTokenSilently(e3)];
          case 2:
            return n2.sent(), [3, 4];
          case 3:
            if (t2 = n2.sent(), !Co.includes(t2.error))
              throw t2;
            return [3, 4];
          case 4:
            return [2];
        }
      });
    });
  }, e2.prototype.getTokenSilently = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, o2, a2, c2 = this;
      return i(this, function(i2) {
        return t2 = n(n({ audience: this.options.audience, ignoreCache: false }, e3), { scope: Qo(this.defaultScope, this.scope, e3.scope) }), o2 = t2.ignoreCache, a2 = r(t2, ["ignoreCache"]), [2, (s2 = function() {
          return c2._getTokenSilently(n({ ignoreCache: o2 }, a2));
        }, u2 = this.options.client_id + "::" + a2.audience + "::" + a2.scope, l2 = Si[u2], l2 || (l2 = s2().finally(function() {
          delete Si[u2], l2 = null;
        }), Si[u2] = l2), l2)];
        var s2, u2, l2;
      });
    });
  }, e2.prototype._getTokenSilently = function(e3) {
    return void 0 === e3 && (e3 = {}), o(this, void 0, void 0, function() {
      var t2, a2, c2, s2, u2, l2, f2 = this;
      return i(this, function(d2) {
        switch (d2.label) {
          case 0:
            return t2 = e3.ignoreCache, a2 = r(e3, ["ignoreCache"]), c2 = function() {
              return o(f2, void 0, void 0, function() {
                var e4;
                return i(this, function(t3) {
                  switch (t3.label) {
                    case 0:
                      return [4, this.cacheManager.get(new $o({ scope: a2.scope, audience: a2.audience || "default", client_id: this.options.client_id }), 60)];
                    case 1:
                      return [2, (e4 = t3.sent()) && e4.access_token];
                  }
                });
              });
            }, t2 ? [3, 2] : [4, c2()];
          case 1:
            if (s2 = d2.sent())
              return [2, s2];
            d2.label = 2;
          case 2:
            return [4, (h2 = function() {
              return _i.acquireLock("auth0.lock.getTokenSilently", 5e3);
            }, p2 = 10, void 0 === p2 && (p2 = 3), o(void 0, void 0, void 0, function() {
              var e4;
              return i(this, function(t3) {
                switch (t3.label) {
                  case 0:
                    e4 = 0, t3.label = 1;
                  case 1:
                    return e4 < p2 ? [4, h2()] : [3, 4];
                  case 2:
                    if (t3.sent())
                      return [2, true];
                    t3.label = 3;
                  case 3:
                    return e4++, [3, 1];
                  case 4:
                    return [2, false];
                }
              });
            }))];
          case 3:
            if (!d2.sent())
              return [3, 15];
            d2.label = 4;
          case 4:
            return d2.trys.push([4, , 12, 14]), t2 ? [3, 6] : [4, c2()];
          case 5:
            if (s2 = d2.sent())
              return [2, s2];
            d2.label = 6;
          case 6:
            return this.options.useRefreshTokens ? [4, this._getTokenUsingRefreshToken(a2)] : [3, 8];
          case 7:
            return l2 = d2.sent(), [3, 10];
          case 8:
            return [4, this._getTokenFromIFrame(a2)];
          case 9:
            l2 = d2.sent(), d2.label = 10;
          case 10:
            return u2 = l2, [4, this.cacheManager.set(n({ client_id: this.options.client_id }, u2))];
          case 11:
            return d2.sent(), this.cookieStorage.save("auth0.is.authenticated", true, { daysUntilExpire: this.sessionCheckExpiryDays }), [2, u2.access_token];
          case 12:
            return [4, _i.releaseLock("auth0.lock.getTokenSilently")];
          case 13:
            return d2.sent(), [7];
          case 14:
            return [3, 16];
          case 15:
            throw new jo();
          case 16:
            return [2];
        }
        var h2, p2;
      });
    });
  }, e2.prototype.getTokenWithPopup = function(e3, t2) {
    return void 0 === e3 && (e3 = {}), void 0 === t2 && (t2 = {}), o(this, void 0, void 0, function() {
      return i(this, function(r2) {
        switch (r2.label) {
          case 0:
            return e3.audience = e3.audience || this.options.audience, e3.scope = Qo(this.defaultScope, this.scope, e3.scope), t2 = n(n({}, xo), t2), [4, this.loginWithPopup(e3, t2)];
          case 1:
            return r2.sent(), [4, this.cacheManager.get(new $o({ scope: e3.scope, audience: e3.audience || "default", client_id: this.options.client_id }))];
          case 2:
            return [2, r2.sent().access_token];
        }
      });
    });
  }, e2.prototype.isAuthenticated = function() {
    return o(this, void 0, void 0, function() {
      return i(this, function(e3) {
        switch (e3.label) {
          case 0:
            return [4, this.getUser()];
          case 1:
            return [2, !!e3.sent()];
        }
      });
    });
  }, e2.prototype.buildLogoutUrl = function(e3) {
    void 0 === e3 && (e3 = {}), null !== e3.client_id ? e3.client_id = e3.client_id || this.options.client_id : delete e3.client_id;
    var t2 = e3.federated, n2 = r(e3, ["federated"]), o2 = t2 ? "&federated" : "";
    return this._url("/v2/logout?" + Xo(n2)) + o2;
  }, e2.prototype.logout = function(e3) {
    var t2 = this;
    void 0 === e3 && (e3 = {});
    var n2 = e3.localOnly, o2 = r(e3, ["localOnly"]);
    if (n2 && o2.federated)
      throw new Error("It is invalid to set both the `federated` and `localOnly` options to `true`");
    var i2 = function() {
      if (t2.cookieStorage.remove("auth0.is.authenticated"), !n2) {
        var e4 = t2.buildLogoutUrl(o2);
        window.location.assign(e4);
      }
    };
    if (this.options.cache)
      return this.cacheManager.clear().then(function() {
        return i2();
      });
    this.cacheManager.clearSync(), i2();
  }, e2.prototype._getTokenFromIFrame = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, o2, a2, c2, s2, u2, l2, f2, d2, h2, p2, y2, v2, m2, g2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return t2 = Vo(Zo()), o2 = Vo(Zo()), a2 = Zo(), [4, No(a2)];
          case 1:
            c2 = i2.sent(), s2 = zo(c2), u2 = this._getParams(e3, t2, o2, s2, e3.redirect_uri || this.options.redirect_uri || window.location.origin), l2 = this._authorizeUrl(n(n({}, u2), { prompt: "none", response_mode: "web_message" })), f2 = e3.timeoutInSeconds || this.options.authorizeTimeoutInSeconds, i2.label = 2;
          case 2:
            return i2.trys.push([2, 5, , 6]), [4, (b2 = l2, w2 = this.domainUrl, S2 = f2, void 0 === S2 && (S2 = 60), new Promise(function(e4, t3) {
              var n2 = window.document.createElement("iframe");
              n2.setAttribute("width", "0"), n2.setAttribute("height", "0"), n2.style.display = "none";
              var r2, o3 = function() {
                window.document.body.contains(n2) && (window.document.body.removeChild(n2), window.removeEventListener("message", r2, false));
              }, i3 = setTimeout(function() {
                t3(new jo()), o3();
              }, 1e3 * S2);
              r2 = function(n3) {
                if (n3.origin == w2 && n3.data && "authorization_response" === n3.data.type) {
                  var a3 = n3.source;
                  a3 && a3.close(), n3.data.response.error ? t3(Lo.fromPayload(n3.data.response)) : e4(n3.data.response), clearTimeout(i3), window.removeEventListener("message", r2, false), setTimeout(o3, 2e3);
                }
              }, window.addEventListener("message", r2, false), window.document.body.appendChild(n2), n2.setAttribute("src", b2);
            }))];
          case 3:
            if (d2 = i2.sent(), t2 !== d2.state)
              throw new Error("Invalid state");
            return h2 = e3.scope, p2 = e3.audience, y2 = r(e3, ["scope", "audience", "redirect_uri", "ignoreCache", "timeoutInSeconds"]), [4, Ho(n(n(n({}, this.customOptions), y2), { scope: h2, audience: p2, baseUrl: this.domainUrl, client_id: this.options.client_id, code_verifier: a2, code: d2.code, grant_type: "authorization_code", redirect_uri: u2.redirect_uri, auth0Client: this.options.auth0Client, useFormData: this.options.useFormData }), this.worker)];
          case 4:
            return v2 = i2.sent(), m2 = this._verifyIdToken(v2.id_token, o2), [2, n(n({}, v2), { decodedToken: m2, scope: u2.scope, audience: u2.audience || "default" })];
          case 5:
            throw "login_required" === (g2 = i2.sent()).error && this.logout({ localOnly: true }), g2;
          case 6:
            return [2];
        }
        var b2, w2, S2;
      });
    });
  }, e2.prototype._getTokenUsingRefreshToken = function(e3) {
    return o(this, void 0, void 0, function() {
      var t2, o2, a2, c2, s2, u2, l2, f2, d2;
      return i(this, function(i2) {
        switch (i2.label) {
          case 0:
            return e3.scope = Qo(this.defaultScope, this.options.scope, e3.scope), [4, this.cacheManager.get(new $o({ scope: e3.scope, audience: e3.audience || "default", client_id: this.options.client_id }))];
          case 1:
            return (t2 = i2.sent()) && t2.refresh_token || this.worker ? [3, 3] : [4, this._getTokenFromIFrame(e3)];
          case 2:
            return [2, i2.sent()];
          case 3:
            o2 = e3.redirect_uri || this.options.redirect_uri || window.location.origin, c2 = e3.scope, s2 = e3.audience, u2 = r(e3, ["scope", "audience", "ignoreCache", "timeoutInSeconds"]), l2 = "number" == typeof e3.timeoutInSeconds ? 1e3 * e3.timeoutInSeconds : null, i2.label = 4;
          case 4:
            return i2.trys.push([4, 6, , 9]), [4, Ho(n(n(n(n(n({}, this.customOptions), u2), { audience: s2, scope: c2, baseUrl: this.domainUrl, client_id: this.options.client_id, grant_type: "refresh_token", refresh_token: t2 && t2.refresh_token, redirect_uri: o2 }), l2 && { timeout: l2 }), { auth0Client: this.options.auth0Client, useFormData: this.options.useFormData }), this.worker)];
          case 5:
            return a2 = i2.sent(), [3, 9];
          case 6:
            return "The web worker is missing the refresh token" === (f2 = i2.sent()).message || f2.message && f2.message.indexOf("invalid refresh token") > -1 ? [4, this._getTokenFromIFrame(e3)] : [3, 8];
          case 7:
            return [2, i2.sent()];
          case 8:
            throw f2;
          case 9:
            return d2 = this._verifyIdToken(a2.id_token), [2, n(n({}, a2), { decodedToken: d2, scope: e3.scope, audience: e3.audience || "default" })];
        }
      });
    });
  }, e2;
}();
var Ei = function() {
};
function xi(e2) {
  return o(this, void 0, void 0, function() {
    var t2;
    return i(this, function(n2) {
      switch (n2.label) {
        case 0:
          return [4, (t2 = new Oi(e2)).checkSession()];
        case 1:
          return n2.sent(), [2, t2];
      }
    });
  });
}
var auth0_spa_js_production_esm_default = xi;
export {
  Oi as Auth0Client,
  Fo as AuthenticationError,
  Lo as GenericError,
  ti as InMemoryCache,
  ei as LocalStorageCache,
  Ko as PopupCancelledError,
  Uo as PopupTimeoutError,
  jo as TimeoutError,
  Ei as User,
  auth0_spa_js_production_esm_default as default
};
/*! Bundled license information:

@auth0/auth0-spa-js/dist/auth0-spa-js.production.esm.js:
  (*! *****************************************************************************
  Copyright (c) Microsoft Corporation.
  
  Permission to use, copy, modify, and/or distribute this software for any
  purpose with or without fee is hereby granted.
  
  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
  REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
  AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
  INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
  LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
  OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
  PERFORMANCE OF THIS SOFTWARE.
  ***************************************************************************** *)
*/
//# sourceMappingURL=@auth0_auth0-spa-js.js.map
