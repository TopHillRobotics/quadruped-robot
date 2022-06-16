class qrComPlanner {
    public:
        qrComPlanner();
        virtual ~qrComPlanner() = default;
        virtual void Reset(float currentTime);
        virtual void Update(float currentTime);
    private:
};