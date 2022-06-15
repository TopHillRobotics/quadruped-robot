class qrFootholdPlanner {
    public:
        qrFootholdPlanner();
        virtual ~qrFootholdPlanner() = default;
        virtual void Reset(float currentTime);
        virtual void Update(float currentTime);
    private:
};