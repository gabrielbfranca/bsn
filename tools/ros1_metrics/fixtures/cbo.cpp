class Supplier { public: void work() {} };
class Consumer { Supplier *s; public: void run() { s->work(); } };
class SignatureUser { public: void use(Supplier &s) { s.work(); } };
class Standalone { public: void work() {} };
