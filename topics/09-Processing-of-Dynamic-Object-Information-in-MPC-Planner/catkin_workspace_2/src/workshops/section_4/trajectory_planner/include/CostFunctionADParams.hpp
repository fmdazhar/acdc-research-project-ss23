#pragma once

//#ifdef CPPADCG

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief Cost Function with Auto-Diff support and passive parameters
 *
 * Custom AD cost function supporting passive parameters.
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR = double>
class CostFunctionADParams : public CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef core::DerivativesCppadJIT<STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1> JacCG;
    typedef typename JacCG::CG_SCALAR CGScalar;
    typedef Eigen::Matrix<CGScalar, 1, 1> MatrixCg;

    typedef Eigen::Matrix<SCALAR, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR, CONTROL_DIM, STATE_DIM> control_state_matrix_t;

    typedef core::StateVector<STATE_DIM, SCALAR> state_vector_t;
    typedef Eigen::Matrix<SCALAR, PARAMS_DIM, 1> params_vector_t;
    typedef core::ControlVector<CONTROL_DIM, SCALAR> control_vector_t;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, 1> VectorXs;
    typedef Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

    /**
     * \brief Basic constructor
     */
    CostFunctionADParams();

    /**
     * Deep-cloning of cost function
     * @return base pointer to clone
     */
    CostFunctionADParams* clone() const override;

    /**
     * \brief Copy constructor
     * @param arg cost function to copy
     */
    CostFunctionADParams(const CostFunctionADParams& arg);


    /**
     * \brief Destructor
     */
    virtual ~CostFunctionADParams();

    /**
     * @brief      Initializes the AD costfunction, generates and compiles
     *             source code
     */
    virtual void initialize() override;

    /**
     * \brief Add an intermediate, auto-differentiable term
     *
     * Use this function to add an auto-differentiable, intermediate term to the cost function.
     *
     * @param term The term to be added
     * @param verbose Flag enabling printouts
     * @return
     */
    void addIntermediateADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    void addIntermediateADTermParam(std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false);

    /**
     * \brief Add a final, auto-differentiable term
     *
     * Use this function to add an auto-differentiable, final term to the cost function.
     *
     * @param term The term to be added
     * @param verbose Flag enabling printouts
     * @return
     */
    void addFinalADTerm(std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false) override;

    void addFinalADTermParam(std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
        bool verbose = false);

    void setCurrentStateAndControl(const state_vector_t& x, const control_vector_t& u, const SCALAR& t = 0.0) override;

    void setCurrentParameters(const params_vector_t& p);

    SCALAR evaluateIntermediate() override;
    SCALAR evaluateTerminal() override;

    state_vector_t stateDerivativeIntermediate() override;
    state_vector_t stateDerivativeTerminal() override;

    control_vector_t controlDerivativeIntermediate() override;
    control_vector_t controlDerivativeTerminal() override;

    state_matrix_t stateSecondDerivativeIntermediate() override;
    state_matrix_t stateSecondDerivativeTerminal() override;

    control_matrix_t controlSecondDerivativeIntermediate() override;
    control_matrix_t controlSecondDerivativeTerminal() override;

    control_state_matrix_t stateControlDerivativeIntermediate() override;
    control_state_matrix_t stateControlDerivativeTerminal() override;


private:
    MatrixCg evaluateIntermediateCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateInputTime);
    MatrixCg evaluateTerminalCg(const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateInputTime);

    //! combined state, control, time and parameter vector
    Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1> stateParamsControlTime_;

    //! intermediate AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>>            intermediateTerms_;
    std::vector<std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>> intermediateTermsParam_;
    //! final AD terms
    std::vector<std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>>            finalTerms_;
    std::vector<std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>> finalTermsParam_;

    //! generated jacobians
    std::shared_ptr<JacCG> intermediateCostCodegen_;
    std::shared_ptr<JacCG> finalCostCodegen_;

    //! cppad functions
    typename JacCG::FUN_TYPE_CG intermediateFun_;
    typename JacCG::FUN_TYPE_CG finalFun_;
};

}  // namespace optcon
}  // namespace ct

//#endif


//#ifdef CPPADCG

namespace ct {
namespace optcon {

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::CostFunctionADParams()
    : CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(),
      stateParamsControlTime_(Eigen::Matrix<SCALAR, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>::Zero())
{
    intermediateFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime) {
        return this->evaluateIntermediateCg(stateParamsInputTime);
    };

    finalFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime) {
        return this->evaluateTerminalCg(stateParamsInputTime);
    };

    intermediateCostCodegen_ = std::shared_ptr<JacCG>(new JacCG(intermediateFun_, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1));
    finalCostCodegen_ = std::shared_ptr<JacCG>(new JacCG(finalFun_, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1));

    setCurrentStateAndControl(this->x_, this->u_, this->t_);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::CostFunctionADParams(const CostFunctionADParams& arg)
    : CostFunctionQuadratic<STATE_DIM, CONTROL_DIM, SCALAR>(arg),
      stateParamsControlTime_(arg.stateParamsControlTime_),
      intermediateFun_(arg.intermediateFun_),
      finalFun_(arg.finalFun_)
{
    intermediateTerms_.resize(arg.intermediateTerms_.size());
    intermediateTermsParam_.resize(arg.intermediateTermsParam_.size());
    finalTerms_.resize(arg.finalTerms_.size());
    finalTermsParam_.resize(arg.finalTermsParam_.size());

    for (size_t i = 0; i < intermediateTerms_.size(); ++i)
        intermediateTerms_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.intermediateTerms_[i]->clone());

    for (size_t i = 0; i < intermediateTermsParam_.size(); ++i)
        intermediateTermsParam_[i] =
            std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.intermediateTermsParam_[i]->clone());

    for (size_t i = 0; i < finalTerms_.size(); ++i)
        finalTerms_[i] =
            std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.finalTerms_[i]->clone());

    for (size_t i = 0; i < finalTermsParam_.size(); ++i)
        finalTermsParam_[i] =
            std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>>(arg.finalTermsParam_[i]->clone());

    intermediateCostCodegen_ = std::shared_ptr<JacCG>(arg.intermediateCostCodegen_->clone());
    finalCostCodegen_ = std::shared_ptr<JacCG>(arg.finalCostCodegen_->clone());
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::~CostFunctionADParams()
{
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>* CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::clone() const
{
    return new CostFunctionADParams(*this);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::initialize()
{
    intermediateFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime) {
        return this->evaluateIntermediateCg(stateParamsInputTime);
    };

    finalFun_ = [&](const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime) {
        return this->evaluateTerminalCg(stateParamsInputTime);
    };

    intermediateCostCodegen_->update(intermediateFun_, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1);
    finalCostCodegen_->update(finalFun_, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1);

    //! @ todo: this should probably become an option (eg. IPOPT can work without cost Hessians)
    ct::core::DerivativesCppadSettings settings;
    settings.createForwardZero_ = true;
    settings.createJacobian_ = true;
    settings.createHessian_ = true;

    finalCostCodegen_->compileJIT(settings, "finalCosts");
    intermediateCostCodegen_->compileJIT(settings, "intermediateCosts");
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::addIntermediateADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    intermediateTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as intermediate AD term" << std::endl;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::addIntermediateADTermParam(
    std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    intermediateTermsParam_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as intermediate AD term with passive parameters" << std::endl;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM,  size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::addFinalADTerm(
    std::shared_ptr<TermBase<STATE_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    finalTerms_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as final AD term" << std::endl;
    }
}

template <size_t STATE_DIM, size_t CONTROL_DIM,  size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::addFinalADTermParam(
    std::shared_ptr<TermBase<STATE_DIM+PARAMS_DIM, CONTROL_DIM, SCALAR, CGScalar>> term,
    bool verbose)
{
    finalTermsParam_.push_back(term);

    if (verbose)
    {
        std::cout << term->getName() + " added as final AD term with passive parameters" << std::endl;
    }
}

// set state and control
template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::setCurrentStateAndControl(const state_vector_t& x,
    const control_vector_t& u,
    const SCALAR& t)
{
    this->x_ = x;
    this->u_ = u;
    this->t_ = t;

    stateParamsControlTime_.segment(0, STATE_DIM) = x;
    stateParamsControlTime_.segment(STATE_DIM+PARAMS_DIM, CONTROL_DIM) = u;
    stateParamsControlTime_(STATE_DIM + PARAMS_DIM + CONTROL_DIM) = t;
    //stateParamsControlTime_ << x, u, t;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
void CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::setCurrentParameters(const params_vector_t& p) {
    stateParamsControlTime_.segment(STATE_DIM, PARAMS_DIM) = p;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::MatrixCg
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::evaluateIntermediateCg(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : intermediateTerms_)
        y += it->evaluateCppadCg(stateParamsInputTime.segment(0, STATE_DIM), stateParamsInputTime.segment(STATE_DIM+PARAMS_DIM, CONTROL_DIM),
            stateParamsInputTime(STATE_DIM + PARAMS_DIM + CONTROL_DIM));

    for (auto it : intermediateTermsParam_)
        y += it->evaluateCppadCg(stateParamsInputTime.segment(0, STATE_DIM+PARAMS_DIM), stateParamsInputTime.segment(STATE_DIM+PARAMS_DIM, CONTROL_DIM),
          stateParamsInputTime(STATE_DIM + PARAMS_DIM + CONTROL_DIM));


    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::MatrixCg
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::evaluateTerminalCg(
    const Eigen::Matrix<CGScalar, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM, 1>& stateParamsInputTime)
{
    CGScalar y = CGScalar(0.0);

    for (auto it : finalTerms_)
        y += it->evaluateCppadCg(stateParamsInputTime.segment(0, STATE_DIM), stateParamsInputTime.segment(STATE_DIM+PARAMS_DIM, CONTROL_DIM),
            stateParamsInputTime(STATE_DIM + PARAMS_DIM + CONTROL_DIM));

    for (auto it : finalTermsParam_)
        y += it->evaluateCppadCg(stateParamsInputTime.segment(0, STATE_DIM+PARAMS_DIM), stateParamsInputTime.segment(STATE_DIM+PARAMS_DIM, CONTROL_DIM),
            stateParamsInputTime(STATE_DIM + PARAMS_DIM + CONTROL_DIM));

    Eigen::Matrix<CGScalar, 1, 1> out;
    out << y;
    return out;
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
SCALAR CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::evaluateIntermediate()
{
    return this->evaluateIntermediateBase() + intermediateCostCodegen_->forwardZero(stateParamsControlTime_)(0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
SCALAR CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::evaluateTerminal()
{
    return this->evaluateTerminalBase() + finalCostCodegen_->forwardZero(stateParamsControlTime_)(0);
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::state_vector_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM> jacTot =
        intermediateCostCodegen_->jacobian(stateParamsControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::state_vector_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM> jacTot = finalCostCodegen_->jacobian(stateParamsControlTime_);
    return jacTot.template leftCols<STATE_DIM>().transpose() + this->stateDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_vector_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::controlDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM> jacTot =
        intermediateCostCodegen_->jacobian(stateParamsControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM+PARAMS_DIM).transpose() + this->controlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_vector_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::controlDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, STATE_DIM + CONTROL_DIM + 1 + PARAMS_DIM> jacTot = finalCostCodegen_->jacobian(stateParamsControlTime_);
    return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM+PARAMS_DIM).transpose() + this->controlDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::state_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateSecondDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::state_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0) + this->stateSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::controlSecondDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM+PARAMS_DIM, STATE_DIM+PARAMS_DIM) +
           this->controlSecondDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::controlSecondDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM+PARAMS_DIM, STATE_DIM+PARAMS_DIM) +
           this->controlSecondDerivativeTerminalBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_state_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateControlDerivativeIntermediate()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = intermediateCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM+PARAMS_DIM, 0) + this->stateControlDerivativeIntermediateBase();
}

template <size_t STATE_DIM, size_t CONTROL_DIM, size_t PARAMS_DIM, typename SCALAR>
typename CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::control_state_matrix_t
CostFunctionADParams<STATE_DIM, CONTROL_DIM, PARAMS_DIM, SCALAR>::stateControlDerivativeTerminal()
{
    Eigen::Matrix<SCALAR, 1, 1> w;
    w << SCALAR(1.0);
    MatrixXs hesTot = finalCostCodegen_->hessian(stateParamsControlTime_, w);
    return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM+PARAMS_DIM, 0) + this->stateControlDerivativeTerminalBase();
}

}  // namespace optcon
}  // namespace ct

//#endif
