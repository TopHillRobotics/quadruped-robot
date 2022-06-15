class qrGaitGenerator {
    public:
        qrGaitGenerator();
        virtual ~qrGaitGenerator() = default;
        virtual void Reset(float currentTime);
        virtual void Update(float currentTime);
    private:
};