#pragma once
namespace fixtures {
class SharedState { int x; public: void a() { x=1; } void b() { x=2; } };
class SplitState { int x,y; public: SplitState():x(0),y(0){} void a() { x=1; } void b() { y=2; } };
class CallBridge { int x; public: void a() { x=1; } void b() { a(); } };
class OnlyConstructor { public: OnlyConstructor() {} };
class Stateless { public: int a() { return 1; } int b() { return 2; } };
class SeparateGetters { int x,y; public: int a() { return x; } int b() { return y; } };
class Branch { public: int linear(int x); int branch(int x); virtual int abstract()=0; };
class B;
class A { B *b; };
class B { A *a; };
class Base {};
class Derived : public Base { A *a; };
}
namespace second { class Branch { public: int inlineMethod() { return 0; } }; }
